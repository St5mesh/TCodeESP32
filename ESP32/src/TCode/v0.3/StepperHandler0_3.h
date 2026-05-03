/* MIT License

Copyright (c) 2024 Jason C. Fain

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

// StepperHandler0_3
// Drives two MKS SERVO42C closed-loop stepper motors for the OSR device using
// the FastAccelStepper library.  Motors are wired via STEP/DIR/EN signals.
// Implements OSR mixing logic:
//   LeftTarget  =  (stroke + roll)
//   RightTarget = (-stroke + roll)
// with a gear ratio of 3:1 (20T:60T) and 120° total arc → ±1600 steps from centre.

#pragma once

#include <Arduino.h>
#include <FastAccelStepper.h>

#include "TCode0_3.h"
#include "SettingsHandler.h"
#include "Global.h"
#include "MotorHandler0_3.h"
#include "TagHandler.h"
#include "settingsFactory.h"
#include "pinMap.h"
#include "settingConstants.h"
#include "constants.h"

class StepperHandler0_3 : public MotorHandler0_3 {

public:
    StepperHandler0_3() : MotorHandler0_3(new TCode0_3()) { }

    void setup() override {
        LogHandler::debug(_TAG, "Setting up stepper handler v0.3");

        m_settingsFactory = SettingsFactory::getInstance();

        // Register TCode axes – OSR uses stroke (L0) and roll (R1).
        m_tcode->setup(FIRMWARE_VERSION_NAME);
        m_tcode->RegisterAxis("L0", "Up");
        m_tcode->RegisterAxis("R1", "Roll");

        // Read motion parameters from settings with safe defaults.
        int accel = (int)STEPPER_ACCELERATION_DEFAULT;
        int maxSpeed = (int)STEPPER_MAX_SPEED_DEFAULT;
        int stepRange = STEPPER_STEP_RANGE_DEFAULT;
        m_settingsFactory->getValue(STEPPER_ACCELERATION, accel);
        m_settingsFactory->getValue(STEPPER_MAX_SPEED, maxSpeed);
        m_settingsFactory->getValue(STEPPER_STEP_RANGE, stepRange);
        m_stepRange = stepRange;

        // Retrieve pin map.
        PinMapOSRStepper* pinMap = PinMapOSRStepper::getInstance();

        m_rightStepPin   = pinMap->rightStep();
        m_rightDirPin    = pinMap->rightDir();
        m_leftStepPin    = pinMap->leftStep();
        m_leftDirPin     = pinMap->leftDir();
        m_enablePin      = pinMap->stepperEnable();

        LogHandler::info(_TAG, "Stepper pins: R_STEP=%d R_DIR=%d L_STEP=%d L_DIR=%d EN=%d",
            m_rightStepPin, m_rightDirPin, m_leftStepPin, m_leftDirPin, m_enablePin);

        if (m_rightStepPin < 0 || m_leftStepPin < 0) {
            LogHandler::error(_TAG, "Invalid stepper step pins");
            m_initFailed = true;
            m_tcode->sendMessage("Stepper init error!");
            return;
        }

        // Configure shared enable pin – active LOW to enable drivers.
        if (m_enablePin >= 0) {
            pinMode(m_enablePin, OUTPUT);
            digitalWrite(m_enablePin, LOW);
        }

        // Initialise FastAccelStepper engine (uses ESP32 RMT/MCPWM hardware, no loop needed).
        m_engine.init();

        m_rightStepper = m_engine.stepperConnectToPin(m_rightStepPin);
        if (!m_rightStepper) {
            LogHandler::error(_TAG, "Failed to connect right stepper to pin %d", m_rightStepPin);
            m_initFailed = true;
            m_tcode->sendMessage("Stepper init error!");
            return;
        }
        m_rightStepper->setDirectionPin(m_rightDirPin);
        m_rightStepper->setSpeedInHz((uint32_t)maxSpeed);
        m_rightStepper->setAcceleration((uint32_t)accel);
        m_rightStepper->setCurrentPosition(0);

        m_leftStepper = m_engine.stepperConnectToPin(m_leftStepPin);
        if (!m_leftStepper) {
            LogHandler::error(_TAG, "Failed to connect left stepper to pin %d", m_leftStepPin);
            m_initFailed = true;
            m_tcode->sendMessage("Stepper init error!");
            return;
        }
        m_leftStepper->setDirectionPin(m_leftDirPin);
        m_leftStepper->setSpeedInHz((uint32_t)maxSpeed);
        m_leftStepper->setAcceleration((uint32_t)accel);
        m_leftStepper->setCurrentPosition(0);

        // Set common (valve/vibe/etc) channels via the base class helper.
        setupCommon();

        m_tcode->sendMessage("Ready!");
        LogHandler::info(_TAG, "Stepper handler setup complete. stepRange=±%d", m_stepRange);
    }

    void setMessageCallback(TCODE_FUNCTION_PTR_T function) override {
        m_tcode->setMessageCallback(function);
    }

    void read(const String &input) override {
        m_tcode->read(input);
    }

    void read(const char* input, size_t len) override {
        for (size_t i = 0; i < len; i++) {
            read(input[i]);
        }
    }

    void read(byte input) override {
        m_tcode->read(input);
    }

    void execute() override {
        if (m_initFailed) {
            return;
        }

        // Read L0 (stroke: 0-9999, centre=5000) and R1 (roll: 0-9999, centre=5000).
        int strokeTcode = channelRead("L0");
        int rollTcode   = channelRead("R1");

        if (strokeTcode < 0) strokeTcode = TCODE_MID;
        if (rollTcode   < 0) rollTcode   = TCODE_MID;
        // Map TCode 0-9999 → -stepRange to +stepRange (centre=0).
        int32_t stroke = map(strokeTcode, TCODE_MIN, TCODE_MAX, -m_stepRange, m_stepRange);
        int32_t roll   = map(rollTcode,   TCODE_MIN, TCODE_MAX, -m_stepRange, m_stepRange);

        if (m_settingsFactory->getInverseStroke()) stroke = -stroke;

        // OSR mixing: left arm = stroke + roll, right arm = -stroke + roll.
        int32_t leftTarget  =  stroke + roll;
        int32_t rightTarget = -stroke + roll;

        // Constrain to physical step range.
        leftTarget  = constrain(leftTarget,  -m_stepRange, m_stepRange);
        rightTarget = constrain(rightTarget, -m_stepRange, m_stepRange);

        m_leftStepper->moveTo(leftTarget);
        m_rightStepper->moveTo(rightTarget);

        // Execute common auxiliary channels (valve, vibe, twist, etc.).
        executeCommon(strokeTcode);
    }

private:
    const char* _TAG = TagHandler::MotorHandler;

    SettingsFactory* m_settingsFactory = nullptr;
    bool m_initFailed = false;

    int8_t m_rightStepPin  = -1;
    int8_t m_rightDirPin   = -1;
    int8_t m_leftStepPin   = -1;
    int8_t m_leftDirPin    = -1;
    int8_t m_enablePin     = -1;

    int32_t m_stepRange = STEPPER_STEP_RANGE_DEFAULT;

    FastAccelStepperEngine m_engine;
    FastAccelStepper*      m_rightStepper = nullptr;
    FastAccelStepper*      m_leftStepper  = nullptr;
};
