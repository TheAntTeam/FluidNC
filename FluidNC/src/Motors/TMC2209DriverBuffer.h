// Copyright (c) 2020 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "TrinamicUartDriver.h"
#include "../Pin.h"
#include "../PinMapper.h"

#include <cstdint>
#include <cstring>

const float TMC2209_RSENSE_DEFAULT = 0.11f;

pinnum_t uartBufferPin;
bool uart_buffer_pin_initialized = false;
Pin * _ub_pin = nullptr;
Uart * ub_ptr;


class TMC2209StepperBuffer : public TMC2209Stepper {
    public:
        // pinnum_t uartBufferPin;
        // Pin ub_pin;
        // String ub_pin_name;
        int read_request_delay;

        TMC2209StepperBuffer(Uart * _uart, float _r_sense, uint8_t _addr, Pin * _uart_buffer_pin) : 
        TMC2209Stepper(_uart, _r_sense, _addr) {
            log_info("Uart buffer pin: " + _uart_buffer_pin->name()) 
            if (!uart_buffer_pin_initialized && _uart_buffer_pin != nullptr) {
                ub_ptr = _uart;
                uartBufferPin = _uart_buffer_pin->undefined() ? -1 : _uart_buffer_pin->getNative
                (Pin::Capabilities::Output);
                _ub_pin = _uart_buffer_pin;
                _ub_pin->setAttr(Pin::Attr::Output);
                log_info("Uart buffer pin initialized: " + _ub_pin->name()) 
                uart_buffer_pin_initialized = true;
            }
            read_request_delay = round((1000000 * (34 + 6)) / ub_ptr->_baud);
            // log_info("read_request_delay = " + String(read_request_delay));
        }

        uint64_t _sendDatagram_buffered(uint8_t datagram[], const uint8_t len, uint16_t timeout) {
            while (available() > 0) serial_read(); // Flush

            #if defined(ARDUINO_ARCH_AVR)
                if (RXTX_pin > 0) {
                    digitalWrite(RXTX_pin, HIGH);
                    pinMode(RXTX_pin, OUTPUT);
                }
            #endif

            _ub_pin->synchronousWrite(false);

            for(int i=0; i<=len; i++) serial_write(datagram[i]);


            #if defined(ARDUINO_ARCH_AVR)
                if (RXTX_pin > 0) {
                    pinMode(RXTX_pin, INPUT_PULLUP);
                }
            #endif

            //delayMicroseconds(350);
            delayMicroseconds(read_request_delay);

            _ub_pin->synchronousWrite(true);

            delay(this->replyDelay);

            // scan for the rx frame and read it
            uint32_t ms = millis();
            uint32_t sync_target = (static_cast<uint32_t>(datagram[0])<<16) | 0xFF00 | datagram[2];
            uint32_t sync = 0;

            do {
                uint32_t ms2 = millis();
                if (ms2 != ms) {
                    // 1ms tick
                    ms = ms2;
                    timeout--;
                }
                if (!timeout) return 0;

                int16_t res = serial_read();
                if (res < 0) continue;

                sync <<= 8;
                sync |= res & 0xFF;
                sync &= 0xFFFFFF;

            } while (sync != sync_target);

            uint64_t out = sync;
            ms = millis();
            timeout = this->abort_window;

            for(uint8_t i=0; i<5;) {
                uint32_t ms2 = millis();
                if (ms2 != ms) {
                    // 1ms tick
                    ms = ms2;
                    timeout--;
                }
                if (!timeout) return 0;

                int16_t res = serial_read();
                if (res < 0) continue;

                out <<= 8;
                out |= res & 0xFF;

                i++;
            }

            #if defined(ARDUINO_ARCH_AVR)
                if (RXTX_pin > 0) {
                    digitalWrite(RXTX_pin, HIGH);
                    pinMode(RXTX_pin, OUTPUT);
                }
            #endif

            while (available() > 0) serial_read(); // Flush

            return out;
        }

        void write(uint8_t addr, uint32_t regVal) override {
            uint8_t len = 7;
            addr |= TMC_WRITE;
            uint8_t datagram[] = {TMC2208_SYNC, slave_address, addr, (uint8_t)(regVal>>24), (uint8_t)(regVal>>16), (uint8_t)(regVal>>8), (uint8_t)(regVal>>0), 0x00};

            datagram[len] = calcCRC(datagram, len);

            _ub_pin->synchronousWrite(false);
            // _ub_pin->write(true);

            for(uint8_t i=0; i<=len; i++) {
                bytesWritten += serial_write(datagram[i]);
            }
            // _ub_pin->synchronousWrite(true);
            // _ub_pin->write(false); 

            delay(replyDelay);
            // _ub_pin->synchronousWrite(true);
        }

        uint32_t read(uint8_t addr) override {
            constexpr uint8_t len = 3;
            addr |= TMC_READ;
            uint8_t datagram[] = {TMC2208_SYNC, slave_address, addr, 0x00};
            datagram[len] = calcCRC(datagram, len);
            uint64_t out = 0x00000000UL;
            

            for (uint8_t i = 0; i < max_retries; i++) {
                // preReadCommunication();
                out = _sendDatagram_buffered(datagram, len, abort_window);
                // postReadCommunication();

                delay(replyDelay);

                CRCerror = false;
                uint8_t out_datagram[] = {
                    static_cast<uint8_t>(out>>56),
                    static_cast<uint8_t>(out>>48),
                    static_cast<uint8_t>(out>>40),
                    static_cast<uint8_t>(out>>32),
                    static_cast<uint8_t>(out>>24),
                    static_cast<uint8_t>(out>>16),
                    static_cast<uint8_t>(out>> 8),
                    static_cast<uint8_t>(out>> 0)
                };
                uint8_t crc = calcCRC(out_datagram, 7);
                if ((crc != static_cast<uint8_t>(out)) || crc == 0 ) {
                    CRCerror = true;
                    out = 0;
                } else {
                    break;
                }
            }

            return out>>8;
        }


};


namespace MotorDrivers {

    class TMC2209DriverBuffer : public TrinamicUartDriver {
    public:
		// Name of the configurable. Must match the name registered in the cpp file.
        //const char* name() const override { return "tmc_2209_buffered"; } override error name() not virtual
        const char* _name = "tmc_2209_buffered";
        TMC2209DriverBuffer(const char* name) : TrinamicUartDriver(name) {}
		
        Pin  _uart_buffer_pin;

        // Overrides for inherited methods
        void init() override;
        void set_disable(bool disable);
        void config_motor() override;
        void debug_message() override;
        void validate() override { StandardStepper::validate(); }

        void group(Configuration::HandlerBase& handler) override {
            TrinamicUartDriver::group(handler);

            handler.item("run_mode", _run_mode, trinamicModes);
            handler.item("homing_mode", _homing_mode, trinamicModes);
            handler.item("homing_amps", _homing_current, 0.0, 10.0);
            handler.item("stallguard", _stallguard, 0, 255);
            handler.item("stallguard_debug", _stallguardDebugMode);
            handler.item("toff_coolstep", _toff_coolstep, 2, 15);
			
            handler.item("uart_buffer_pin", _uart_buffer_pin);
        }

        void afterParse() override {
            TrinamicUartDriver::afterParse();
            if (_homing_current == 0) {
                _homing_current = _run_current;
                log_warn(axisName() << " " << name() << " homing current not in config. Using run current");
            }
        }

    private:
        TMC2209StepperBuffer* tmc2209 = nullptr;

        bool test();
        void set_registers(bool isHoming);
    };
}
