/*
 * name:        SX1276_77_78_79
 * description: 137 MHz to 1020 MHz Low Power Long Range Transceiver featuring the LoRa (TM) long range modem
 * manuf:       Semtech
 * version:     0.1
 * url:         http://www.semtech.com/images/datasheet/sx1276_77_78_79.pdf
 * date:        2016-08-01
 * author       https://chisl.io/
 * file:        SX1276_77_78_79_FSK.hpp
 */

#include <cinttypes>

/* Derive from class SX1276_77_78_79_FSK_Base and implement the read and write functions! */

/* SX1276_77_78_79: 137 MHz to 1020 MHz Low Power Long Range Transceiver featuring the LoRa (TM) long range modem */
class SX1276_77_78_79_FSK_Base
{
public:
	/* Pure virtual functions that need to be implemented in derived class: */
	virtual uint8_t read8(uint16_t address, uint16_t n=8) = 0;  // 8 bit read
	virtual void write(uint16_t address, uint8_t value, uint16_t n=8) = 0;  // 8 bit write
	virtual uint16_t read16(uint16_t address, uint16_t n=16) = 0;  // 16 bit read
	virtual void write(uint16_t address, uint16_t value, uint16_t n=16) = 0;  // 16 bit write
	virtual uint32_t read32(uint16_t address, uint16_t n=32) = 0;  // 32 bit read
	virtual void write(uint16_t address, uint32_t value, uint16_t n=32) = 0;  // 32 bit write
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                             REG Fifo                                              *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG Fifo:
	 * FIFO data input/output 
	 */
	struct Fifo
	{
		static const uint16_t __address = 0;
		
		/* Bits Fifo: */
		struct Fifo_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00000000; // 8'h0
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register Fifo */
	void setFifo(uint8_t value)
	{
		write(Fifo::__address, value, 8);
	}
	
	/* Get register Fifo */
	uint8_t getFifo()
	{
		return read8(Fifo::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                            REG OpMode                                             *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG OpMode:
	 */
	struct OpMode
	{
		static const uint16_t __address = 1;
		
		/* Bits LongRangeMode: */
		/*
		 * This bit can be modified only in Sleep mode.
		 *           A write operation on other device modes is ignored. 
		 */
		struct LongRangeMode
		{
			/* Mode:r */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b10000000; // [7]
			static const uint8_t FSK_OOK_MODE = 0b0; // FSK/OOK Mode
			static const uint8_t LORA_MODE = 0b1; // LoRaTM Mode
		};
		/* Bits ModulationType: */
		/*
		 * Modulation scheme: 
		 * 10 = 11 = reserved 
		 */
		struct ModulationType
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00; // 2'd0
			static const uint8_t mask = 0b01100000; // [5,6]
			static const uint8_t FSK = 0b00; // 
			static const uint8_t OOK = 0b01; // 
		};
		/* Bits reserved_0: */
		struct reserved_0
		{
			/* Mode:r */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00010000; // [4]
		};
		/* Bits LowFrequencyModeOn: */
		/* Access Low Frequency Mode registers (from address 0x61 on)  */
		struct LowFrequencyModeOn
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b1; // 1'b1
			static const uint8_t mask = 0b00001000; // [3]
			static const uint8_t HIGH_FREQUENCY_MODE = 0b0; // High Frequency Mode (access to HF test registers)
			static const uint8_t LOW_FREQUENCY_MODE = 0b1; // Low Frequency Mode (access to LF test registers) §
		};
		/* Bits Mode: */
		/*
		 * Transceiver modes 
		 * 110 = reserved
		 *           111 = reserved 
		 */
		struct Mode
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b001; // 3'b1
			static const uint8_t mask = 0b00000111; // [0,1,2]
			static const uint8_t SLEEP = 0b00; // Sleep mode
			static const uint8_t STDBY = 0b01; // Stdby mode
			static const uint8_t FSTx = 0b10; // FS mode TX
			static const uint8_t Tx = 0b11; // Transmitter mode
			static const uint8_t FSRx = 0b100; // FS mode RX
			static const uint8_t Rx = 0b101; // Receiver mode
		};
	};
	
	/* Set register OpMode */
	void setOpMode(uint8_t value)
	{
		write(OpMode::__address, value, 8);
	}
	
	/* Get register OpMode */
	uint8_t getOpMode()
	{
		return read8(OpMode::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG BitrateMsb                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG BitrateMsb:
	 * MSB of Bit Rate (chip rate if Manchester encoding is enabled) 
	 */
	struct BitrateMsb
	{
		static const uint16_t __address = 2;
		
		/* Bits BitrateMsb: */
		struct BitrateMsb_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00011010; // 8'h1a
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register BitrateMsb */
	void setBitrateMsb(uint8_t value)
	{
		write(BitrateMsb::__address, value, 8);
	}
	
	/* Get register BitrateMsb */
	uint8_t getBitrateMsb()
	{
		return read8(BitrateMsb::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG BitrateLsb                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG BitrateLsb:
	 * LSB of bit rate (chip rate if Manchester encoding is enabled)
	 *       BitRate =
	 *       Default value: 4.8 kb/s 
	 */
	struct BitrateLsb
	{
		static const uint16_t __address = 3;
		
		/* Bits BitrateLsb: */
		struct BitrateLsb_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00001011; // 8'hb
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register BitrateLsb */
	void setBitrateLsb(uint8_t value)
	{
		write(BitrateLsb::__address, value, 8);
	}
	
	/* Get register BitrateLsb */
	uint8_t getBitrateLsb()
	{
		return read8(BitrateLsb::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                             REG Fdev                                              *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG Fdev:
	 * LSB of the frequency deviation Fdev = Fstep ? Fdev(15,0) Default value: 5 kHz 
	 */
	struct Fdev
	{
		static const uint16_t __address = 4;
		
		/* Bits reserved_0: */
		struct reserved_0
		{
			/* Mode:rw */
			static const uint16_t dflt = 0b00; // 2'd0
			static const uint16_t mask = 0b1100000000000000; // [14,15]
		};
		/* Bits Fdev: */
		/* frequency deviation  */
		struct Fdev_
		{
			/* Mode:rw */
			static const uint16_t dflt = 0b00000001010010; // 14'h52
			static const uint16_t mask = 0b0011111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13]
		};
	};
	
	/* Set register Fdev */
	void setFdev(uint16_t value)
	{
		write(Fdev::__address, value, 16);
	}
	
	/* Get register Fdev */
	uint16_t getFdev()
	{
		return read16(Fdev::__address, 16);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                            REG FrfLsb                                             *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG FrfLsb:
	 * RF carrier frequency
	 *       Default value: 434.000 MHz
	 *       The RF frequency is taken into account internally only when:
	 *       - entering FSRX/FSTX modes
	 *       - re-starting the receiver 
	 */
	struct FrfLsb
	{
		static const uint16_t __address = 6;
		
		/* Bits FrfLsb: */
		struct FrfLsb_
		{
			/* Mode:rw */
			static const uint32_t dflt = 0b011011001000000000000000; // 24'h6c8000
			static const uint32_t mask = 0b111111111111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23]
		};
	};
	
	/* Set register FrfLsb */
	void setFrfLsb(uint32_t value)
	{
		write(FrfLsb::__address, value, 24);
	}
	
	/* Get register FrfLsb */
	uint32_t getFrfLsb()
	{
		return read32(FrfLsb::__address, 24);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG PaConfig                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG PaConfig:
	 */
	struct PaConfig
	{
		static const uint16_t __address = 9;
		
		/* Bits PaSelect: */
		/* Selects PA output pin  */
		struct PaSelect
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b10000000; // [7]
			static const uint8_t RFO = 0b0; // RFO pin. Maximum power of +14 dBm
			static const uint8_t PA_BOOST = 0b1; // PA_BOOST pin. Maximum power of +20 dBm
		};
		/* Bits MaxPower: */
		/* Select max output power: Pmax=10.8+0.6*MaxPower [dBm]  */
		struct MaxPower
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b100; // 3'h4
			static const uint8_t mask = 0b01110000; // [4,5,6]
		};
		/* Bits OutputPower: */
		/*
		 * Pout=Pmax-(15-OutputPower) if PaSelect = 0 (RFO pins)
		 *           Pout=17-(15-OutputPower)   if PaSelect = 1 (PA_BOOST pin) 
		 */
		struct OutputPower
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b1111; // 4'hf
			static const uint8_t mask = 0b00001111; // [0,1,2,3]
		};
	};
	
	/* Set register PaConfig */
	void setPaConfig(uint8_t value)
	{
		write(PaConfig::__address, value, 8);
	}
	
	/* Get register PaConfig */
	uint8_t getPaConfig()
	{
		return read8(PaConfig::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                            REG PaRamp                                             *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG PaRamp:
	 */
	struct PaRamp
	{
		static const uint16_t __address = 10;
		
		/* Bits unused_0: */
		struct unused_0
		{
			/* Mode:r */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b10000000; // [7]
		};
		/* Bits ModulationShaping: */
		/*
		 * Data shaping: In FSK:
		 *           - 00 = no shaping
		 *           - 01 = Gaussian filter BT = 1.0
		 *           - 10 = Gaussian filter BT = 0.5
		 *           - 11 = Gaussian filter BT = 0.3 
		 *           In OOK:
		 *           - 00 = no shaping
		 *           - 01 = filtering with fcutoff = bit_rate
		 *           - 10 = filtering with fcutoff = 2*bit_rate (for bit_rate < 125 kb/s) 
		 *           - 11 = reserved 
		 */
		struct ModulationShaping
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00; // 2'd0
			static const uint8_t mask = 0b01100000; // [5,6]
		};
		/* Bits reserved_1: */
		struct reserved_1
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00010000; // [4]
		};
		/* Bits PaRamp: */
		/* Rise/Fall time of ramp up/down in FSK 0000 = 3.4 ms  */
		struct PaRamp_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b1001; // 4'h9
			static const uint8_t mask = 0b00001111; // [0,1,2,3]
			static const uint8_t t_2_ms = 0b001; // 
			static const uint8_t t_1_ms = 0b010; // 
			static const uint8_t t_500_us = 0b011; // 
			static const uint8_t t_250_us = 0b100; // 
			static const uint8_t t_125_us = 0b101; // 
			static const uint8_t t_100_us = 0b110; // 
			static const uint8_t t_62_us = 0b111; // 
			static const uint8_t t_50_us = 0b1000; // 
			static const uint8_t t_40_us = 0b1001; // 
			static const uint8_t t_31_us = 0b1010; // 
			static const uint8_t t_25_us = 0b1011; // 
			static const uint8_t t_20_us = 0b1100; // 
			static const uint8_t t_15_us = 0b1101; // 
			static const uint8_t t_12_us = 0b1110; // 
			static const uint8_t t_10_us = 0b1111; // 
		};
	};
	
	/* Set register PaRamp */
	void setPaRamp(uint8_t value)
	{
		write(PaRamp::__address, value, 8);
	}
	
	/* Get register PaRamp */
	uint8_t getPaRamp()
	{
		return read8(PaRamp::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                             REG Ocp                                              *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/* REG Ocp:
	 */
	struct Ocp
	{
		static const uint16_t __address = 11;
		
		/* Bits unused_0: */
		struct unused_0
		{
			/* Mode:r */
			static const uint8_t dflt = 0b00; // 2'd0
			static const uint8_t mask = 0b11000000; // [6,7]
		};
		/* Bits OcpOn: */
		/* Enables overload current protection (OCP) for the PA:  */
		struct OcpOn
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b1; // 1'd1
			static const uint8_t mask = 0b00100000; // [5]
			static const uint8_t DISABLED = 0b0; // 0 = OCP disabled
			static const uint8_t ENABLED = 0b1; // 1 = OCP enabled
		};
		/* Bits OcpTrim: */
		/*
		 * Trimming of OCP current:
		 *           Imax = 45+5*OcpTrim [mA] if OcpTrim <= 15 (120 mA) /
		 *           Imax = -30+10*OcpTrim [mA] if 15 < OcpTrim <= 27 (130 to 240 mA)
		 *           Imax = 240mA for higher settings Default Imax = 100mA 
		 */
		struct OcpTrim
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b01011; // 5'hb
			static const uint8_t mask = 0b00011111; // [0,1,2,3,4]
		};
	};
	
	/* Set register Ocp */
	void setOcp(uint8_t value)
	{
		write(Ocp::__address, value, 8);
	}
	
	/* Get register Ocp */
	uint8_t getOcp()
	{
		return read8(Ocp::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                             REG Lna                                              *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/* REG Lna:
	 */
	struct Lna
	{
		static const uint16_t __address = 12;
		
		/* Bits LnaGain: */
		/*
		 * LNA gain setting: 000 = reserved 
		 * 'b111 = reserved Note:
		 *           Reading this address always returns the current LNA gain
		 *           (which may be different from what had been previously selected if AGC is enabled. 
		 */
		struct LnaGain
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b001; // 3'b1
			static const uint8_t mask = 0b11100000; // [5,6,7]
			static const uint8_t G1 = 0b01; // highest gain
			static const uint8_t G2 = 0b10; // highest gain - 6 dB
			static const uint8_t G3 = 0b11; // highest gain - 12 dB
			static const uint8_t G4 = 0b100; // highest gain - 24 dB
			static const uint8_t G5 = 0b101; // highest gain - 36 dB
			static const uint8_t G6 = 0b110; // highest gain - 48 dB
		};
		/* Bits LnaBoostLf: */
		/*
		 * Low Frequency (RFI_LF) LNA current adjustment 00 = Default LNA current
		 *           Other = Reserved 
		 */
		struct LnaBoostLf
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00; // 2'd0
			static const uint8_t mask = 0b00011000; // [3,4]
		};
		/* Bits reserved_0: */
		struct reserved_0
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00000100; // [2]
		};
		/* Bits LnaBoostHf: */
		/*
		 * High Frequency (RFI_HF) LNA current adjustment 00 = Default LNA current
		 *           11 = Boost on, 150% LNA current 
		 */
		struct LnaBoostHf
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00; // 2'd0
			static const uint8_t mask = 0b00000011; // [0,1]
		};
	};
	
	/* Set register Lna */
	void setLna(uint8_t value)
	{
		write(Lna::__address, value, 8);
	}
	
	/* Get register Lna */
	uint8_t getLna()
	{
		return read8(Lna::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG RxConfig                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG RxConfig:
	 */
	struct RxConfig
	{
		static const uint16_t __address = 13;
		
		/* Bits RestartRxOnCollision: */
		/*
		 * Turns on the mechanism restarting the receiver automatically if it gets
		 *           saturated or a packet collision is detected
		 *           0 = No automatic Restart
		 *           1 = Automatic restart On 
		 */
		struct RestartRxOnCollision
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b10000000; // [7]
		};
		/* Bits RestartRxWithoutPllLock: */
		/*
		 * Triggers a manual Restart of the Receiver chain when set to 1.
		 *           Use this bit when there is no frequency change, RestartRxWithPllLock otherwise. 
		 */
		struct RestartRxWithoutPllLock
		{
			/* Mode:wt */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b01000000; // [6]
		};
		/* Bits RestartRxWithPllLock: */
		/*
		 * Triggers a manual Restart of the Receiver chain when set to 1.
		 *           Use this bit when there is a frequency change, requiring some time for the PLL to re-lock. 
		 */
		struct RestartRxWithPllLock
		{
			/* Mode:wt */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00100000; // [5]
		};
		/* Bits AfcAutoOn: */
		/*
		 * 0 = No AFC performed at receiver startup
		 *           1 = AFC is performed at each receiver startup 
		 */
		struct AfcAutoOn
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00010000; // [4]
		};
		/* Bits AgcAutoOn: */
		/* 0 = LNA gain forced by the LnaGain Setting 1 = LNA gain is controlled by the AGC  */
		struct AgcAutoOn
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b1; // 1'd1
			static const uint8_t mask = 0b00001000; // [3]
		};
		/* Bits RxTrigger: */
		/* Selects the event triggering AGC and/or AFC at receiver startup. See Table 24 for a description.  */
		struct RxTrigger
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b110; // 3'd6
			static const uint8_t mask = 0b00000111; // [0,1,2]
		};
	};
	
	/* Set register RxConfig */
	void setRxConfig(uint8_t value)
	{
		write(RxConfig::__address, value, 8);
	}
	
	/* Get register RxConfig */
	uint8_t getRxConfig()
	{
		return read8(RxConfig::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG RssiConfig                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG RssiConfig:
	 */
	struct RssiConfig
	{
		static const uint16_t __address = 14;
		
		/* Bits RssiOffset: */
		/*
		 * Signed RSSI offset, to compensate for the possible losses/gains in the
		 *           front-end (LNA, SAW filter...)
		 *           1dB / LSB, 2's complement format 
		 */
		struct RssiOffset
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00000; // 5'd0
			static const uint8_t mask = 0b11111000; // [3,4,5,6,7]
		};
		/* Bits RssiSmoothing: */
		/* Defines the number of samples taken to average the RSSI result:  */
		struct RssiSmoothing
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b010; // 3'd2
			static const uint8_t mask = 0b00000111; // [0,1,2]
			static const uint8_t S_2 = 0b00; // 
			static const uint8_t S_4 = 0b01; // 
			static const uint8_t S_8 = 0b10; // 
			static const uint8_t S_16 = 0b11; // 
			static const uint8_t S_32 = 0b100; // 
			static const uint8_t S_64 = 0b101; // 
			static const uint8_t S_128 = 0b110; // 
			static const uint8_t S_256 = 0b111; // 
		};
	};
	
	/* Set register RssiConfig */
	void setRssiConfig(uint8_t value)
	{
		write(RssiConfig::__address, value, 8);
	}
	
	/* Get register RssiConfig */
	uint8_t getRssiConfig()
	{
		return read8(RssiConfig::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                        REG RssiCollision                                         *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG RssiCollision:
	 * Sets the threshold used to consider that an interferer is detected,
	 *       witnessing a packet collision. 1dB/LSB (only RSSI increase) Default: 10dB 
	 */
	struct RssiCollision
	{
		static const uint16_t __address = 15;
		
		/* Bits RssiCollision: */
		struct RssiCollision_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00001010; // 8'ha
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register RssiCollision */
	void setRssiCollision(uint8_t value)
	{
		write(RssiCollision::__address, value, 8);
	}
	
	/* Get register RssiCollision */
	uint8_t getRssiCollision()
	{
		return read8(RssiCollision::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG RssiThresh                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG RssiThresh:
	 * RSSI trigger level for the Rssi interrupt:
	 *       - RssiThreshold / 2 [dBm] 
	 */
	struct RssiThresh
	{
		static const uint16_t __address = 16;
		
		/* Bits RssiThresh: */
		struct RssiThresh_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b11111111; // 8'hff
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register RssiThresh */
	void setRssiThresh(uint8_t value)
	{
		write(RssiThresh::__address, value, 8);
	}
	
	/* Get register RssiThresh */
	uint8_t getRssiThresh()
	{
		return read8(RssiThresh::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG RssiValue                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG RssiValue:
	 * Absolute value of the RSSI in dBm, 0.5dB steps. RSSI = - RssiValue/2 [dBm] 
	 */
	struct RssiValue
	{
		static const uint16_t __address = 17;
		
		/* Bits RssiValue: */
		struct RssiValue_
		{
			/* Mode:r */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register RssiValue */
	void setRssiValue(uint8_t value)
	{
		write(RssiValue::__address, value, 8);
	}
	
	/* Get register RssiValue */
	uint8_t getRssiValue()
	{
		return read8(RssiValue::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                             REG RxBw                                              *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG RxBw:
	 */
	struct RxBw
	{
		static const uint16_t __address = 18;
		
		/* Bits unused_0: */
		struct unused_0
		{
			/* Mode:r */
			static const uint8_t mask = 0b10000000; // [7]
		};
		/* Bits reserved_1: */
		struct reserved_1
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00; // 2'd0
			static const uint8_t mask = 0b01100000; // [5,6]
		};
		/* Bits RxBwMant: */
		/*
		 * Channel filter bandwidth control: 
		 * 11 = reserved 
		 */
		struct RxBwMant
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b10; // 2'd2
			static const uint8_t mask = 0b00011000; // [3,4]
			static const uint8_t BW_16 = 0b00; // 
			static const uint8_t BW_24 = 0b10; // 
			static const uint8_t BW_20 = 0b01; // 
		};
		/* Bits RxBwExp: */
		/* Channel filter bandwidth control  */
		struct RxBwExp
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b101; // 3'h5
			static const uint8_t mask = 0b00000111; // [0,1,2]
		};
	};
	
	/* Set register RxBw */
	void setRxBw(uint8_t value)
	{
		write(RxBw::__address, value, 8);
	}
	
	/* Get register RxBw */
	uint8_t getRxBw()
	{
		return read8(RxBw::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                            REG AfcBw                                             *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/* REG AfcBw:
	 */
	struct AfcBw
	{
		static const uint16_t __address = 19;
		
		/* Bits reserved_0: */
		struct reserved_0
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b000; // 3'd0
			static const uint8_t mask = 0b11100000; // [5,6,7]
		};
		/* Bits RxBwMantAfc: */
		/* RxBwMant parameter used during the AFC  */
		struct RxBwMantAfc
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b01; // 2'd1
			static const uint8_t mask = 0b00011000; // [3,4]
		};
		/* Bits RxBwExpAfc: */
		/* RxBwExp parameter used during the AFC  */
		struct RxBwExpAfc
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b011; // 3'd3
			static const uint8_t mask = 0b00000111; // [0,1,2]
		};
	};
	
	/* Set register AfcBw */
	void setAfcBw(uint8_t value)
	{
		write(AfcBw::__address, value, 8);
	}
	
	/* Get register AfcBw */
	uint8_t getAfcBw()
	{
		return read8(AfcBw::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                           REG OokPeak                                            *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/* REG OokPeak:
	 */
	struct OokPeak
	{
		static const uint16_t __address = 20;
		
		/* Bits reserved_0: */
		struct reserved_0
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00; // 2'd0
			static const uint8_t mask = 0b11000000; // [6,7]
		};
		/* Bits BitSyncOn: */
		/*
		 * Enables the Bit Synchronizer.
		 *           0 = Bit Sync disabled (not possible in Packet mode) 1 = Bit Sync enabled 
		 */
		struct BitSyncOn
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b1; // 1'd1
			static const uint8_t mask = 0b00100000; // [5]
		};
		/* Bits OokThreshType: */
		/*
		 * Selects the type of threshold in the OOK data slicer: 
		 * 11 = reserved 
		 */
		struct OokThreshType
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b01; // 2'd1
			static const uint8_t mask = 0b00011000; // [3,4]
			static const uint8_t FIXED = 0b00; // fixed threshold
			static const uint8_t AVG = 0b10; // average mode
			static const uint8_t PEAK = 0b01; // peak mode (default)
		};
		/* Bits OokPeakTheshStep: */
		/* Size of each decrement of the RSSI threshold in the OOK demodulator:  */
		struct OokPeakTheshStep
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b000; // 3'd0
			static const uint8_t mask = 0b00000111; // [0,1,2]
			static const uint8_t THRESH_0_5 = 0b00; // 
			static const uint8_t THRESH_1_0 = 0b01; // 
			static const uint8_t THRESH_1_5 = 0b10; // 
			static const uint8_t THRESH_2_0 = 0b11; // 
			static const uint8_t THRESH_3_0 = 0b100; // 
			static const uint8_t THRESH_4_0 = 0b101; // 
			static const uint8_t THRESH_5_0 = 0b110; // 
			static const uint8_t THRESH_6_0 = 0b111; // 
		};
	};
	
	/* Set register OokPeak */
	void setOokPeak(uint8_t value)
	{
		write(OokPeak::__address, value, 8);
	}
	
	/* Get register OokPeak */
	uint8_t getOokPeak()
	{
		return read8(OokPeak::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                            REG OokFix                                             *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG OokFix:
	 * Fixed threshold for the Data Slicer in OOK mode
	 *       Floor threshold for the Data Slicer in OOK when Peak mode is used 
	 */
	struct OokFix
	{
		static const uint16_t __address = 21;
		
		/* Bits OokFix: */
		struct OokFix_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00001100; // 8'hc
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register OokFix */
	void setOokFix(uint8_t value)
	{
		write(OokFix::__address, value, 8);
	}
	
	/* Get register OokFix */
	uint8_t getOokFix()
	{
		return read8(OokFix::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                            REG OokAvg                                             *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG OokAvg:
	 */
	struct OokAvg
	{
		static const uint16_t __address = 22;
		
		/* Bits OokPeakThreshDec: */
		/*
		 * Period of decrement of the RSSI threshold in the OOK demodulator:
		 *           000 = once per chip
		 *           001 = once every 2 chips
		 *           010 = once every 4 chips
		 *           011 = once every 8 chips
		 *           100 = twice in each chip
		 *           101 = 4 times in each chip
		 *           110 = 8 times in each chip
		 *           111 = 16 times in each chip 
		 */
		struct OokPeakThreshDec
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b000; // 3'd0
			static const uint8_t mask = 0b11100000; // [5,6,7]
		};
		/* Bits reserved_0: */
		/* reserved  */
		struct reserved_0
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b1; // 1'd1
			static const uint8_t mask = 0b00010000; // [4]
		};
		/* Bits OokAverageOffset: */
		/*
		 * Static offset added to the threshold in average mode in order to reduce glitching activity (OOK only):
		 *           00 = 0.0 dB
		 *           01 = 2.0 dB
		 *           10 = 4.0 dB
		 *           11 = 6.0 dB 
		 */
		struct OokAverageOffset
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00; // 2'd0
			static const uint8_t mask = 0b00001100; // [2,3]
		};
		/* Bits OokAverageThreshFilt: */
		/*
		 * Filter coefficients in average mode of the OOK demodulator:
		 *           00 = fC ~ chip rate / 32 pi
		 *           01 = fC ~ chip rate / 8 pi
		 *           10 = fC ~ chip rate / 4 pi
		 *           11 = fC ~ chip rate / 2 pi 
		 */
		struct OokAverageThreshFilt
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b10; // 2'd2
			static const uint8_t mask = 0b00000011; // [0,1]
		};
	};
	
	/* Set register OokAvg */
	void setOokAvg(uint8_t value)
	{
		write(OokAvg::__address, value, 8);
	}
	
	/* Get register OokAvg */
	uint8_t getOokAvg()
	{
		return read8(OokAvg::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                            REG AfcFei                                             *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG AfcFei:
	 */
	struct AfcFei
	{
		static const uint16_t __address = 26;
		
		/* Bits unused_0: */
		struct unused_0
		{
			/* Mode:r */
			static const uint8_t mask = 0b11100000; // [5,6,7]
		};
		/* Bits AgcStart: */
		/* Triggers an AGC sequence when set to 1.  */
		struct AgcStart
		{
			/* Mode:wt */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00010000; // [4]
		};
		/* Bits reserved_1: */
		struct reserved_1
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00001000; // [3]
		};
		/* Bits unused_2: */
		struct unused_2
		{
			static const uint8_t mask = 0b00000100; // [2]
		};
		/* Bits AfcClear: */
		/* Clear AFC register set in Rx mode. Always reads 0.  */
		struct AfcClear
		{
			/* Mode:wc */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00000010; // [1]
		};
		/* Bits AfcAutoClearOn: */
		/*
		 * Only valid if AfcAutoOn is set
		 *           0 = AFC register is not cleared at the beginning of the automatic AFC phase
		 *           1 = AFC register is cleared at the beginning of the automatic AFC phase 
		 */
		struct AfcAutoClearOn
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00000001; // [0]
		};
	};
	
	/* Set register AfcFei */
	void setAfcFei(uint8_t value)
	{
		write(AfcFei::__address, value, 8);
	}
	
	/* Get register AfcFei */
	uint8_t getAfcFei()
	{
		return read8(AfcFei::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                             REG Afc                                              *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG Afc:
	 * AfcValue, 2's complement format. Can be used to overwrite the current AFC value 
	 */
	struct Afc
	{
		static const uint16_t __address = 27;
		
		/* Bits Afc: */
		struct Afc_
		{
			/* Mode:rw */
			static const uint16_t dflt = 0b0000000000000000; // 16'h0
			static const uint16_t mask = 0b1111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
		};
	};
	
	/* Set register Afc */
	void setAfc(uint16_t value)
	{
		write(Afc::__address, value, 16);
	}
	
	/* Get register Afc */
	uint16_t getAfc()
	{
		return read16(Afc::__address, 16);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                             REG Fei                                              *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG Fei:
	 * Measured frequency offset, 2's complement. Must be read before RegFeiLsb.
	 *       Frequency error = FeiValue x Fstep 
	 */
	struct Fei
	{
		static const uint16_t __address = 29;
		
		/* Bits Fei: */
		struct Fei_
		{
			/* Mode:rw */
			static const uint16_t mask = 0b1111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
		};
	};
	
	/* Set register Fei */
	void setFei(uint16_t value)
	{
		write(Fei::__address, value, 16);
	}
	
	/* Get register Fei */
	uint16_t getFei()
	{
		return read16(Fei::__address, 16);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                        REG PreambleDetect                                         *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG PreambleDetect:
	 */
	struct PreambleDetect
	{
		static const uint16_t __address = 31;
		
		/* Bits PreambleDetectorOn: */
		/*
		 * Enables Preamble detector when set to 1. The AGC settings supersede this bit during the startup / AGC phase.
		 *           0 = Turned off
		 *           1 = Turned on 
		 */
		struct PreambleDetectorOn
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b1; // 1'd1
			static const uint8_t mask = 0b10000000; // [7]
		};
		/* Bits PreambleDetectorSize: */
		/*
		 * Number of Preamble bytes to detect to trigger an interrupt
		 *           00 = 1 byte
		 *           10 = 3 bytes
		 *           01 = 2 bytes
		 *           11 = Reserved 
		 */
		struct PreambleDetectorSize
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b01; // 2'd1
			static const uint8_t mask = 0b01100000; // [5,6]
		};
		/* Bits PreambleDetectorTol: */
		/* Number or chip errors tolerated over PreambleDetectorSize. 4 chips per bit.  */
		struct PreambleDetectorTol
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b01010; // 5'ha
			static const uint8_t mask = 0b00011111; // [0,1,2,3,4]
		};
	};
	
	/* Set register PreambleDetect */
	void setPreambleDetect(uint8_t value)
	{
		write(PreambleDetect::__address, value, 8);
	}
	
	/* Get register PreambleDetect */
	uint8_t getPreambleDetect()
	{
		return read8(PreambleDetect::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG RxTimeout1                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG RxTimeout1:
	 * Timeout interrupt is generated TimeoutRxRssi*16*Tbit after switching to Rx mode
	 *       if Rssi interrupt doesn?t occur (i.e. RssiValue > RssiThreshold)
	 *       0x00: TimeoutRxRssi is disabled 
	 */
	struct RxTimeout1
	{
		static const uint16_t __address = 32;
		
		/* Bits RxTimeout1: */
		struct RxTimeout1_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00000000; // 8'h0
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register RxTimeout1 */
	void setRxTimeout1(uint8_t value)
	{
		write(RxTimeout1::__address, value, 8);
	}
	
	/* Get register RxTimeout1 */
	uint8_t getRxTimeout1()
	{
		return read8(RxTimeout1::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG RxTimeout2                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG RxTimeout2:
	 * Timeout interrupt is generated TimeoutRxPreamble*16*Tbit after switching to
	 *       Rx mode if Preamble interrupt doesn't occur
	 *       0x00: TimeoutRxPreamble is disabled 
	 */
	struct RxTimeout2
	{
		static const uint16_t __address = 33;
		
		/* Bits RxTimeout2: */
		struct RxTimeout2_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00000000; // 8'h0
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register RxTimeout2 */
	void setRxTimeout2(uint8_t value)
	{
		write(RxTimeout2::__address, value, 8);
	}
	
	/* Get register RxTimeout2 */
	uint8_t getRxTimeout2()
	{
		return read8(RxTimeout2::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG RxTimeout3                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG RxTimeout3:
	 * Timeout interrupt is generated TimeoutSignalSync*16*Tbit after the Rx mode
	 *       is programmed, if SyncAddress doesn?t occur 0x00: TimeoutSignalSync is disabled 
	 */
	struct RxTimeout3
	{
		static const uint16_t __address = 34;
		
		/* Bits RxTimeout3: */
		struct RxTimeout3_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00000000; // 8'h0
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register RxTimeout3 */
	void setRxTimeout3(uint8_t value)
	{
		write(RxTimeout3::__address, value, 8);
	}
	
	/* Get register RxTimeout3 */
	uint8_t getRxTimeout3()
	{
		return read8(RxTimeout3::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                           REG RxDelay                                            *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG RxDelay:
	 * Additional delay before an automatic receiver restart is launched:
	 *       Delay = InterPacketRxDelay*4*Tbit 
	 */
	struct RxDelay
	{
		static const uint16_t __address = 35;
		
		/* Bits RxDelay: */
		struct RxDelay_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00000000; // 8'h0
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register RxDelay */
	void setRxDelay(uint8_t value)
	{
		write(RxDelay::__address, value, 8);
	}
	
	/* Get register RxDelay */
	uint8_t getRxDelay()
	{
		return read8(RxDelay::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                             REG Osc                                              *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/* REG Osc:
	 */
	struct Osc
	{
		static const uint16_t __address = 36;
		
		/* Bits unused_0: */
		struct unused_0
		{
			/* Mode:r */
			static const uint8_t mask = 0b11110000; // [4,5,6,7]
		};
		/* Bits RcCalStart: */
		/*
		 * Triggers the calibration of the RC oscillator when set. Always reads 0.
		 *           RC calibration must be triggered in Standby mode. 
		 */
		struct RcCalStart
		{
			/* Mode:wt */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00001000; // [3]
		};
		/* Bits ClkOut: */
		/*
		 * Selects CLKOUT frequency:
		 *           000 = FXOSC
		 *           001 = FXOSC / 2
		 *           010 = FXOSC / 4
		 *           011 = FXOSC / 8
		 *           100 = FXOSC / 16
		 *           101 = FXOSC / 32
		 *           110 = RC (automatically enabled)
		 *           111 = OFF 
		 */
		struct ClkOut
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b111; // 3'd7
			static const uint8_t mask = 0b00000111; // [0,1,2]
		};
	};
	
	/* Set register Osc */
	void setOsc(uint8_t value)
	{
		write(Osc::__address, value, 8);
	}
	
	/* Get register Osc */
	uint8_t getOsc()
	{
		return read8(Osc::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG Preamble                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG Preamble:
	 * Size of the preamble to be sent (from TxStartCondition fulfilled). 
	 */
	struct Preamble
	{
		static const uint16_t __address = 37;
		
		/* Bits Preamble: */
		struct Preamble_
		{
			/* Mode:rw */
			static const uint16_t dflt = 0b0000000000000011; // 16'h3
			static const uint16_t mask = 0b1111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
		};
	};
	
	/* Set register Preamble */
	void setPreamble(uint16_t value)
	{
		write(Preamble::__address, value, 16);
	}
	
	/* Get register Preamble */
	uint16_t getPreamble()
	{
		return read16(Preamble::__address, 16);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG SyncConfig                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG SyncConfig:
	 */
	struct SyncConfig
	{
		static const uint16_t __address = 39;
		
		/* Bits AutoRestartRxMode: */
		/*
		 * Controls the automatic restart of the receiver after the reception of a
		 *           valid packet (PayloadReady or CrcOk):
		 *           00 = Off
		 *           01 = On, without waiting for the PLL to re-lock
		 *           10 = On, wait for the PLL to lock (frequency changed)
		 *           11 = reserved 
		 */
		struct AutoRestartRxMode
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b10; // 2'd2
			static const uint8_t mask = 0b11000000; // [6,7]
		};
		/* Bits PreamblePolarity: */
		/*
		 * Sets the polarity of the Preamble
		 *           0 = 0xAA (default)
		 *           1 = 0x55 
		 */
		struct PreamblePolarity
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00100000; // [5]
		};
		/* Bits SyncOn: */
		/*
		 * Enables the Sync word generation and detection:
		 *           0 = Off
		 *           1 = On 
		 */
		struct SyncOn
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b1; // 1'd1
			static const uint8_t mask = 0b00010000; // [4]
		};
		/* Bits reserved_0: */
		struct reserved_0
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00001000; // [3]
		};
		/* Bits SyncSize: */
		/*
		 * Size of the Sync word:
		 *           (SyncSize + 1) bytes, (SyncSize) bytes if ioHomeOn=1 
		 */
		struct SyncSize
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b011; // 3'd3
			static const uint8_t mask = 0b00000111; // [0,1,2]
		};
	};
	
	/* Set register SyncConfig */
	void setSyncConfig(uint8_t value)
	{
		write(SyncConfig::__address, value, 8);
	}
	
	/* Get register SyncConfig */
	uint8_t getSyncConfig()
	{
		return read8(SyncConfig::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG SyncValue1                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG SyncValue1:
	 * 1st byte of Sync word. (MSB byte) Used if SyncOn is set. 
	 */
	struct SyncValue1
	{
		static const uint16_t __address = 40;
		
		/* Bits SyncValue1: */
		struct SyncValue1_
		{
			/* Mode:rw */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register SyncValue1 */
	void setSyncValue1(uint8_t value)
	{
		write(SyncValue1::__address, value, 8);
	}
	
	/* Get register SyncValue1 */
	uint8_t getSyncValue1()
	{
		return read8(SyncValue1::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG SyncValue2                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG SyncValue2:
	 * 2nd byte of Sync word
	 *       Used if SyncOn is set and (SyncSize +1) >= 2. 
	 */
	struct SyncValue2
	{
		static const uint16_t __address = 41;
		
		/* Bits SyncValue2: */
		struct SyncValue2_
		{
			/* Mode:rw */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register SyncValue2 */
	void setSyncValue2(uint8_t value)
	{
		write(SyncValue2::__address, value, 8);
	}
	
	/* Get register SyncValue2 */
	uint8_t getSyncValue2()
	{
		return read8(SyncValue2::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG SyncValue3                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG SyncValue3:
	 * 3rd byte of Sync word.
	 *       Used if SyncOn is set and (SyncSize +1) >= 3. 
	 */
	struct SyncValue3
	{
		static const uint16_t __address = 42;
		
		/* Bits SyncValue3: */
		struct SyncValue3_
		{
			/* Mode:rw */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register SyncValue3 */
	void setSyncValue3(uint8_t value)
	{
		write(SyncValue3::__address, value, 8);
	}
	
	/* Get register SyncValue3 */
	uint8_t getSyncValue3()
	{
		return read8(SyncValue3::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG SyncValue4                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG SyncValue4:
	 * 4th byte of Sync word.
	 *       Used if SyncOn is set and (SyncSize +1) >= 4. 
	 */
	struct SyncValue4
	{
		static const uint16_t __address = 43;
		
		/* Bits SyncValue4: */
		struct SyncValue4_
		{
			/* Mode:rw */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register SyncValue4 */
	void setSyncValue4(uint8_t value)
	{
		write(SyncValue4::__address, value, 8);
	}
	
	/* Get register SyncValue4 */
	uint8_t getSyncValue4()
	{
		return read8(SyncValue4::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG SyncValue5                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG SyncValue5:
	 * 5th byte of Sync word.
	 *       Used if SyncOn is set and (SyncSize +1) >= 5. 
	 */
	struct SyncValue5
	{
		static const uint16_t __address = 44;
		
		/* Bits SyncValue5: */
		struct SyncValue5_
		{
			/* Mode:rw */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register SyncValue5 */
	void setSyncValue5(uint8_t value)
	{
		write(SyncValue5::__address, value, 8);
	}
	
	/* Get register SyncValue5 */
	uint8_t getSyncValue5()
	{
		return read8(SyncValue5::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG SyncValue6                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG SyncValue6:
	 * 6th byte of Sync word.
	 *       Used if SyncOn is set and (SyncSize +1) >= 6. 
	 */
	struct SyncValue6
	{
		static const uint16_t __address = 45;
		
		/* Bits SyncValue6: */
		struct SyncValue6_
		{
			/* Mode:rw */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register SyncValue6 */
	void setSyncValue6(uint8_t value)
	{
		write(SyncValue6::__address, value, 8);
	}
	
	/* Get register SyncValue6 */
	uint8_t getSyncValue6()
	{
		return read8(SyncValue6::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG SyncValue7                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG SyncValue7:
	 * 7th byte of Sync word.
	 *       Used if SyncOn is set and (SyncSize +1) >= 7. 
	 */
	struct SyncValue7
	{
		static const uint16_t __address = 46;
		
		/* Bits SyncValue7: */
		struct SyncValue7_
		{
			/* Mode:rw */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register SyncValue7 */
	void setSyncValue7(uint8_t value)
	{
		write(SyncValue7::__address, value, 8);
	}
	
	/* Get register SyncValue7 */
	uint8_t getSyncValue7()
	{
		return read8(SyncValue7::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG SyncValue8                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG SyncValue8:
	 * 8th byte of Sync word.
	 *       Used if SyncOn is set and (SyncSize +1) = 8. 
	 */
	struct SyncValue8
	{
		static const uint16_t __address = 47;
		
		/* Bits SyncValue8: */
		struct SyncValue8_
		{
			/* Mode:rw */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register SyncValue8 */
	void setSyncValue8(uint8_t value)
	{
		write(SyncValue8::__address, value, 8);
	}
	
	/* Get register SyncValue8 */
	uint8_t getSyncValue8()
	{
		return read8(SyncValue8::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                         REG PacketConfig                                          *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG PacketConfig:
	 */
	struct PacketConfig
	{
		static const uint16_t __address = 48;
		
		/* Bits PacketFormat: */
		/*
		 * Defines the packet format used:
		 *           0 = Fixed length
		 *           1 = Variable length 
		 */
		struct PacketFormat
		{
			/* Mode:rw */
			static const uint16_t dflt = 0b1; // 1'd1
			static const uint16_t mask = 0b1000000000000000; // [15]
		};
		/* Bits DcFree: */
		/*
		 * Defines DC-free encoding/decoding performed:
		 *           00 = None (Off)
		 *           01 = Manchester
		 *           10 = Whitening
		 *           11 = reserved 
		 */
		struct DcFree
		{
			/* Mode:rw */
			static const uint16_t dflt = 0b00; // 2'd0
			static const uint16_t mask = 0b0110000000000000; // [13,14]
		};
		/* Bits CrcOn: */
		/*
		 * Enables CRC calculation/check (Tx/Rx):
		 *           0 = Off
		 *           1 = On 
		 */
		struct CrcOn
		{
			/* Mode:rw */
			static const uint16_t dflt = 0b1; // 1'd1
			static const uint16_t mask = 0b0001000000000000; // [12]
		};
		/* Bits CrcAutoClearOff: */
		/*
		 * Defines the behavior of the packet handler when CRC check fails:
		 *           0 = Clear FIFO and restart new packet reception. No PayloadReady interrupt issued.
		 *           1 = Do not clear FIFO. PayloadReady interrupt issued. 
		 */
		struct CrcAutoClearOff
		{
			/* Mode:rw */
			static const uint16_t dflt = 0b0; // 1'd0
			static const uint16_t mask = 0b0000100000000000; // [11]
		};
		/* Bits AddressFiltering: */
		/*
		 * Defines address based filtering in Rx:
		 *           00 = None (Off)
		 *           01 = Address field must match NodeAddress
		 *           10 = Address field must match NodeAddress or BroadcastAddress
		 *           11 = reserved 
		 */
		struct AddressFiltering
		{
			/* Mode:rw */
			static const uint16_t dflt = 0b00; // 2'd0
			static const uint16_t mask = 0b0000011000000000; // [9,10]
		};
		/* Bits CrcWhiteningType: */
		/*
		 * Selects the CRC and whitening algorithms:
		 *           0 = CCITT CRC implementation with standard whitening
		 *           1 = IBM CRC implementation with alternate whitening 
		 */
		struct CrcWhiteningType
		{
			/* Mode:rw */
			static const uint16_t dflt = 0b0; // 1'd0
			static const uint16_t mask = 0b0000000100000000; // [8]
		};
		/* Bits unused_0: */
		struct unused_0
		{
			/* Mode:r */
			static const uint16_t mask = 0b0000000010000000; // [7]
		};
		/* Bits DataMode: */
		/*
		 * Data processing mode:
		 *           0 = Continuous mode
		 *           1 = Packet mode 
		 */
		struct DataMode
		{
			/* Mode:rw */
			static const uint16_t dflt = 0b1; // 1'd1
			static const uint16_t mask = 0b0000000001000000; // [6]
		};
		/* Bits IoHomeOn: */
		/*
		 * Enables the io-homecontrol(R) compatibility mode
		 *           0 = Disabled
		 *           1 = Enabled 
		 */
		struct IoHomeOn
		{
			/* Mode:rw */
			static const uint16_t dflt = 0b0; // 1'd0
			static const uint16_t mask = 0b0000000000100000; // [5]
		};
		/* Bits IoHomePowerFrame: */
		/* reserved - Linked to io-homecontrol(R) compatibility mode  */
		struct IoHomePowerFrame
		{
			/* Mode:rw */
			static const uint16_t dflt = 0b0; // 1'd0
			static const uint16_t mask = 0b0000000000010000; // [4]
		};
		/* Bits BeaconOn: */
		/* Enables the Beacon mode in Fixed packet format  */
		struct BeaconOn
		{
			/* Mode:rw */
			static const uint16_t dflt = 0b0; // 1'd0
			static const uint16_t mask = 0b0000000000001000; // [3]
		};
		/* Bits PayloadLength: */
		/* Packet Length Most significant bits (10:8)  */
		struct PayloadLength
		{
			/* Mode:rw */
			static const uint16_t dflt = 0b000; // 3'd0
			static const uint16_t mask = 0b0000000000000111; // [0,1,2]
		};
	};
	
	/* Set register PacketConfig */
	void setPacketConfig(uint16_t value)
	{
		write(PacketConfig::__address, value, 16);
	}
	
	/* Get register PacketConfig */
	uint16_t getPacketConfig()
	{
		return read16(PacketConfig::__address, 16);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                        REG PayloadLength                                         *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG PayloadLength:
	 * If PacketFormat = 0 (fixed), payload length.
	 *       If PacketFormat = 1 (variable), max length in Rx, not used in Tx. 
	 */
	struct PayloadLength
	{
		static const uint16_t __address = 50;
		
		/* Bits PayloadLength: */
		struct PayloadLength_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b01000000; // 8'h40
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register PayloadLength */
	void setPayloadLength(uint8_t value)
	{
		write(PayloadLength::__address, value, 8);
	}
	
	/* Get register PayloadLength */
	uint8_t getPayloadLength()
	{
		return read8(PayloadLength::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG NodeAdrs                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG NodeAdrs:
	 * Node address used in address filtering. 
	 */
	struct NodeAdrs
	{
		static const uint16_t __address = 51;
		
		/* Bits NodeAdrs: */
		struct NodeAdrs_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00000000; // 8'h0
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register NodeAdrs */
	void setNodeAdrs(uint8_t value)
	{
		write(NodeAdrs::__address, value, 8);
	}
	
	/* Get register NodeAdrs */
	uint8_t getNodeAdrs()
	{
		return read8(NodeAdrs::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                        REG BroadcastAdrs                                         *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG BroadcastAdrs:
	 * Broadcast address used in address filtering. 
	 */
	struct BroadcastAdrs
	{
		static const uint16_t __address = 52;
		
		/* Bits BroadcastAdrs: */
		struct BroadcastAdrs_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00000000; // 8'h0
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register BroadcastAdrs */
	void setBroadcastAdrs(uint8_t value)
	{
		write(BroadcastAdrs::__address, value, 8);
	}
	
	/* Get register BroadcastAdrs */
	uint8_t getBroadcastAdrs()
	{
		return read8(BroadcastAdrs::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG FifoThresh                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG FifoThresh:
	 */
	struct FifoThresh
	{
		static const uint16_t __address = 53;
		
		/* Bits TxStartCondition: */
		/*
		 * Defines the condition to start packet transmission:
		 *           0 = FifoLevel (i.e. the number of bytes in the FIFO exceeds FifoThreshold)
		 *           1 = FifoEmpty goes low(i.e. at least one byte in the FIFO) 
		 */
		struct TxStartCondition
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b1; // 1'd1
			static const uint8_t mask = 0b10000000; // [7]
		};
		/* Bits unused_0: */
		struct unused_0
		{
			/* Mode:r */
			static const uint8_t mask = 0b01000000; // [6]
		};
		/* Bits FifoThreshold: */
		/*
		 * Used to trigger FifoLevel interrupt, when: number of bytes in
		 *           FIFO >= FifoThreshold + 1 
		 */
		struct FifoThreshold
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b001111; // 6'hf
			static const uint8_t mask = 0b00111111; // [0,1,2,3,4,5]
		};
	};
	
	/* Set register FifoThresh */
	void setFifoThresh(uint8_t value)
	{
		write(FifoThresh::__address, value, 8);
	}
	
	/* Get register FifoThresh */
	uint8_t getFifoThresh()
	{
		return read8(FifoThresh::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG SeqConfig1                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG SeqConfig1:
	 */
	struct SeqConfig1
	{
		static const uint16_t __address = 54;
		
		/* Bits SequencerStart: */
		/*
		 * Controls the top level Sequencer
		 *           When set to '1', executes the 'Start' transition.
		 *           The sequencer can only be enabled when the chip is in Sleep or Standby mode. 
		 */
		struct SequencerStart
		{
			/* Mode:wt */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b10000000; // [7]
		};
		/* Bits SequencerStop: */
		/* Forces the Sequencer Off. Always reads '0'  */
		struct SequencerStop
		{
			/* Mode:wt */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b01000000; // [6]
		};
		/* Bits IdleMode: */
		/*
		 * Selects chip mode during the state:
		 *           0: Standby mode
		 *           1: Sleep mode 
		 */
		struct IdleMode
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00100000; // [5]
		};
		/* Bits FromStart: */
		/*
		 * Controls the Sequencer transition when SequencerStart is set to 1 in Sleep or Standby mode:
		 *           00: to LowPowerSelection 01: to Receive state
		 *           10: to Transmit state
		 *           11: to Transmit state on a FifoLevel interrupt 
		 */
		struct FromStart
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00; // 2'd0
			static const uint8_t mask = 0b00011000; // [3,4]
		};
		/* Bits LowPowerSelection: */
		/*
		 * Selects the Sequencer LowPower state after a to LowPowerSelection transition:
		 *           0: SequencerOff state with chip on Initial mode
		 *           1: Idle state with chip on Standby or Sleep mode depending on
		 *           IdleMode
		 *           Note:   Initial mode is the chip LowPower mode at Sequencer Start. 
		 */
		struct LowPowerSelection
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00000100; // [2]
		};
		/* Bits FromIdle: */
		/*
		 * Controls the Sequencer transition from the Idle state on a T1 interrupt:
		 *           0: to Transmit state
		 *           1: to Receive state 
		 */
		struct FromIdle
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00000010; // [1]
		};
		/* Bits FromTransmit: */
		/*
		 * Controls the Sequencer transition from the Transmit state:
		 *           0: to LowPowerSelection on a PacketSent interrupt
		 *           1: to Receive state on a PacketSent interrupt 
		 */
		struct FromTransmit
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00000001; // [0]
		};
	};
	
	/* Set register SeqConfig1 */
	void setSeqConfig1(uint8_t value)
	{
		write(SeqConfig1::__address, value, 8);
	}
	
	/* Get register SeqConfig1 */
	uint8_t getSeqConfig1()
	{
		return read8(SeqConfig1::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG SeqConfig2                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG SeqConfig2:
	 */
	struct SeqConfig2
	{
		static const uint16_t __address = 55;
		
		/* Bits FromReceive: */
		/*
		 * Controls the Sequencer transition from the Receive state
		 *           000 and 111: unused
		 *           001: to PacketReceived state on a PayloadReady interrupt
		 *           010: to LowPowerSelection on a PayloadReady interrupt
		 *           011: to PacketReceived state on a CrcOk interrupt (1)
		 *           100: to SequencerOff state on a Rssi interrupt
		 *           101: to SequencerOff state on a SyncAddress interrupt
		 *           110: to SequencerOff state on a PreambleDetect interrupt
		 *           Irrespective of this setting, transition to LowPowerSelection on a T2 interrupt
		 *           (1) If the CRC is wrong (corrupted packet, with CRC on but CrcAutoClearOn=0),
		 *           the PayloadReady interrupt will drive the sequencer to RxTimeout state. 
		 */
		struct FromReceive
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b000; // 3'd0
			static const uint8_t mask = 0b11100000; // [5,6,7]
		};
		/* Bits FromRxTimeout: */
		/*
		 * Controls the state-machine transition from the Receive state on a RxTimeout interrupt (and on PayloadReady if FromReceive = 011):
		 *           00: to Receive State, via ReceiveRestart
		 *           01: to Transmit state
		 *           10: to LowPowerSelection
		 *           11: to SequencerOff state
		 *           Note:   RxTimeout interrupt is a TimeoutRxRssi, TimeoutRxPreamble or
		 *           TimeoutSignalSync interrupt 
		 */
		struct FromRxTimeout
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00; // 2'd0
			static const uint8_t mask = 0b00011000; // [3,4]
		};
		/* Bits FromPacketReceived: */
		/*
		 * Controls the state-machine transition from the PacketReceived state:
		 *           000: to SequencerOff state
		 *           001: to Transmit state on a FifoEmpty interrupt
		 *           010: to LowPowerSelection
		 *           011: to Receive via FS mode, if frequency was changed
		 *           100: to Receive state (no frequency change) 
		 */
		struct FromPacketReceived
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b000; // 3'd0
			static const uint8_t mask = 0b00000111; // [0,1,2]
		};
	};
	
	/* Set register SeqConfig2 */
	void setSeqConfig2(uint8_t value)
	{
		write(SeqConfig2::__address, value, 8);
	}
	
	/* Get register SeqConfig2 */
	uint8_t getSeqConfig2()
	{
		return read8(SeqConfig2::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG TimerResol                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG TimerResol:
	 */
	struct TimerResol
	{
		static const uint16_t __address = 56;
		
		/* Bits unused_0: */
		struct unused_0
		{
			/* Mode:r */
			static const uint8_t mask = 0b11110000; // [4,5,6,7]
		};
		/* Bits Timer1Resolution: */
		/*
		 * Resolution of Timer 1
		 *           00: Timer1 disabled
		 *           01: 64 us
		 *           10: 4.1 ms
		 *           11: 262 ms 
		 */
		struct Timer1Resolution
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00; // 2'd0
			static const uint8_t mask = 0b00001100; // [2,3]
		};
		/* Bits Timer2Resolution: */
		/*
		 * Resolution of Timer 2
		 *           00: Timer2 disabled
		 *           01: 64 us
		 *           10: 4.1 ms
		 *           11: 262 ms 
		 */
		struct Timer2Resolution
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00; // 2'd0
			static const uint8_t mask = 0b00000011; // [0,1]
		};
	};
	
	/* Set register TimerResol */
	void setTimerResol(uint8_t value)
	{
		write(TimerResol::__address, value, 8);
	}
	
	/* Get register TimerResol */
	uint8_t getTimerResol()
	{
		return read8(TimerResol::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG Timer1Coef                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG Timer1Coef:
	 * Multiplying coefficient for Timer 1 
	 */
	struct Timer1Coef
	{
		static const uint16_t __address = 57;
		
		/* Bits Timer1Coef: */
		struct Timer1Coef_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b11110101; // 8'hf5
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register Timer1Coef */
	void setTimer1Coef(uint8_t value)
	{
		write(Timer1Coef::__address, value, 8);
	}
	
	/* Get register Timer1Coef */
	uint8_t getTimer1Coef()
	{
		return read8(Timer1Coef::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG Timer2Coef                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG Timer2Coef:
	 * Multiplying coefficient for Timer 2 
	 */
	struct Timer2Coef
	{
		static const uint16_t __address = 58;
		
		/* Bits Timer2Coef: */
		struct Timer2Coef_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00100000; // 8'h20
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register Timer2Coef */
	void setTimer2Coef(uint8_t value)
	{
		write(Timer2Coef::__address, value, 8);
	}
	
	/* Get register Timer2Coef */
	uint8_t getTimer2Coef()
	{
		return read8(Timer2Coef::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG ImageCal                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG ImageCal:
	 */
	struct ImageCal
	{
		static const uint16_t __address = 59;
		
		/* Bits AutoImageCalOn: */
		/*
		 * Controls the Image calibration mechanism
		 *           0 = Calibration of the receiver depending on the temperature is disabled
		 *           1 = Calibration of the receiver depending on the temperature enabled. 
		 */
		struct AutoImageCalOn
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b10000000; // [7]
		};
		/* Bits ImageCalStart: */
		/* Triggers the IQ and RSSI calibration when set in Standby mode.  */
		struct ImageCalStart
		{
			/* Mode:wt */
			static const uint8_t mask = 0b01000000; // [6]
		};
		/* Bits ImageCalRunning: */
		/*
		 * Set to 1 while the Image and RSSI calibration are running.
		 *           Toggles back to 0 when the process is completed 
		 */
		struct ImageCalRunning
		{
			/* Mode:r */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00100000; // [5]
		};
		/* Bits unused_0: */
		struct unused_0
		{
			/* Mode:r */
			static const uint8_t mask = 0b00010000; // [4]
		};
		/* Bits TempChange: */
		/*
		 * IRQ flag witnessing a temperature change exceeding TempThreshold since
		 *           the last Image and RSSI calibration:
		 *           0 = Temperature change lower than TempThreshold
		 *           1 = Temperature change greater than TempThreshold 
		 */
		struct TempChange
		{
			/* Mode:r */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00001000; // [3]
		};
		/* Bits TempThreshold: */
		/*
		 * Temperature change threshold to trigger a new I/Q calibration
		 *           00 = 5C
		 *           01 = 10C
		 *           10 = 15C
		 *           11 = 20C 
		 */
		struct TempThreshold
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b01; // 2'd1
			static const uint8_t mask = 0b00000110; // [1,2]
		};
		/* Bits TempMonitorOff: */
		/*
		 * Controls the temperature monitor operation:
		 *           0 = Temperature monitoring done in all modes except Sleep and Standby
		 *           1 = Temperature monitoring stopped. 
		 */
		struct TempMonitorOff
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00000001; // [0]
		};
	};
	
	/* Set register ImageCal */
	void setImageCal(uint8_t value)
	{
		write(ImageCal::__address, value, 8);
	}
	
	/* Get register ImageCal */
	uint8_t getImageCal()
	{
		return read8(ImageCal::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                             REG Temp                                              *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG Temp:
	 * Measured temperature
	 *       -1C per Lsb
	 *       Needs calibration for absolute accuracy 
	 */
	struct Temp
	{
		static const uint16_t __address = 60;
		
		/* Bits Temp: */
		struct Temp_
		{
			/* Mode:r */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register Temp */
	void setTemp(uint8_t value)
	{
		write(Temp::__address, value, 8);
	}
	
	/* Get register Temp */
	uint8_t getTemp()
	{
		return read8(Temp::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                            REG LowBat                                             *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG LowBat:
	 */
	struct LowBat
	{
		static const uint16_t __address = 61;
		
		/* Bits unused_0: */
		struct unused_0
		{
			/* Mode:r */
			static const uint8_t mask = 0b11110000; // [4,5,6,7]
		};
		/* Bits LowBatOn: */
		/*
		 * Low Battery detector enable signal
		 *           0 = LowBat detector disabled
		 *           1 = LowBat detector enabled 
		 */
		struct LowBatOn
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00001000; // [3]
		};
		/* Bits LowBatTrim: */
		/*
		 * Trimming of the LowBat threshold:
		 *           000 = 1.695 V
		 *           001 = 1.764 V
		 *           010 = 1.835 V (d)
		 *           011 = 1.905 V
		 *           100 = 1.976 V
		 *           101 = 2.045 V
		 *           110 = 2.116 V
		 *           111 = 2.185 V 
		 */
		struct LowBatTrim
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b010; // 3'd2
			static const uint8_t mask = 0b00000111; // [0,1,2]
		};
	};
	
	/* Set register LowBat */
	void setLowBat(uint8_t value)
	{
		write(LowBat::__address, value, 8);
	}
	
	/* Get register LowBat */
	uint8_t getLowBat()
	{
		return read8(LowBat::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG IrqFlags1                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/* REG IrqFlags1:
	 */
	struct IrqFlags1
	{
		static const uint16_t __address = 62;
		
		/* Bits ModeReady: */
		/*
		 * Set when the operation mode requested in Mode, is ready
		 *           - Sleep: Entering Sleep mode
		 *           - Standby: XO is running
		 *           - FS: PLL is locked
		 *           - Rx: RSSI sampling starts
		 *           - Tx: PA ramp-up completed
		 *           Cleared when changing the operating mode. 
		 */
		struct ModeReady
		{
			/* Mode:r */
			static const uint8_t mask = 0b10000000; // [7]
		};
		/* Bits RxReady: */
		/* Set in Rx mode, after RSSI, AGC and AFC. Cleared when leaving Rx.  */
		struct RxReady
		{
			/* Mode:r */
			static const uint8_t mask = 0b01000000; // [6]
		};
		/* Bits TxReady: */
		/* Set in Tx mode, after PA ramp-up. Cleared when leaving Tx.  */
		struct TxReady
		{
			/* Mode:r */
			static const uint8_t mask = 0b00100000; // [5]
		};
		/* Bits PllLock: */
		/* Set (in FS, Rx or Tx) when the PLL is locked. Cleared when it is not.  */
		struct PllLock
		{
			/* Mode:r */
			static const uint8_t mask = 0b00010000; // [4]
		};
		/* Bits Rssi: */
		/*
		 * Set in Rx when the RssiValue exceeds RssiThreshold.
		 *           Cleared when leaving Rx or setting this bit to 1. 
		 */
		struct Rssi
		{
			/* Mode:rwc */
			static const uint8_t mask = 0b00001000; // [3]
		};
		/* Bits Timeout: */
		/*
		 * Set when a timeout occurs
		 *           Cleared when leaving Rx or FIFO is emptied. 
		 */
		struct Timeout
		{
			/* Mode:r */
			static const uint8_t mask = 0b00000100; // [2]
		};
		/* Bits PreambleDetect: */
		/* Set when the Preamble Detector has found valid Preamble. bit clear when set to 1  */
		struct PreambleDetect
		{
			/* Mode:rwc */
			static const uint8_t mask = 0b00000010; // [1]
		};
		/* Bits SyncAddressMatch: */
		/*
		 * Set when Sync and Address (if enabled) are detected. Cleared when leaving Rx or FIFO is emptied.
		 *           This bit is read only in Packet mode, rwc in Continuous mode 
		 */
		struct SyncAddressMatch
		{
			/* Mode:rwc */
			static const uint8_t mask = 0b00000001; // [0]
		};
	};
	
	/* Set register IrqFlags1 */
	void setIrqFlags1(uint8_t value)
	{
		write(IrqFlags1::__address, value, 8);
	}
	
	/* Get register IrqFlags1 */
	uint8_t getIrqFlags1()
	{
		return read8(IrqFlags1::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG IrqFlags2                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/* REG IrqFlags2:
	 */
	struct IrqFlags2
	{
		static const uint16_t __address = 63;
		
		/* Bits FifoFull: */
		/* Set when FIFO is full (i.e. contains 66 bytes), else cleared.  */
		struct FifoFull
		{
			/* Mode:r */
			static const uint8_t mask = 0b10000000; // [7]
		};
		/* Bits FifoEmpty: */
		/* Set when FIFO is empty, and cleared when there is at least 1 byte in the FIFO.  */
		struct FifoEmpty
		{
			/* Mode:r */
			static const uint8_t mask = 0b01000000; // [6]
		};
		/* Bits FifoLevel: */
		/*
		 * Set when the number of bytes in the FIFO strictly exceeds
		 *           FifoThreshold, else cleared. 
		 */
		struct FifoLevel
		{
			/* Mode:r */
			static const uint8_t mask = 0b00100000; // [5]
		};
		/* Bits FifoOverrun: */
		/*
		 * Set when FIFO overrun occurs. (except in Sleep mode)
		 *           Flag(s) and FIFO are cleared when this bit is set.
		 *           The FIFO then becomes immediately available for the next transmission / reception. 
		 */
		struct FifoOverrun
		{
			/* Mode:rwc */
			static const uint8_t mask = 0b00010000; // [4]
		};
		/* Bits PacketSent: */
		/* Set in Tx when the complete packet has been sent. Cleared when exiting Tx  */
		struct PacketSent
		{
			/* Mode:r */
			static const uint8_t mask = 0b00001000; // [3]
		};
		/* Bits PayloadReady: */
		/*
		 * Set in Rx when the payload is ready (i.e. last byte received and CRC, if
		 *           enabled and CrcAutoClearOff is cleared, is Ok). Cleared when FIFO is empty. 
		 */
		struct PayloadReady
		{
			/* Mode:r */
			static const uint8_t mask = 0b00000100; // [2]
		};
		/* Bits CrcOk: */
		/* Set in Rx when the CRC of the payload is Ok. Cleared when FIFO is empty.  */
		struct CrcOk
		{
			/* Mode:r */
			static const uint8_t mask = 0b00000010; // [1]
		};
		/* Bits LowBat: */
		/*
		 * Set when the battery voltage drops below the Low Battery threshold.
		 *           Cleared only when set to 1 by the user. 
		 */
		struct LowBat
		{
			/* Mode:rwc */
			static const uint8_t mask = 0b00000001; // [0]
		};
	};
	
	/* Set register IrqFlags2 */
	void setIrqFlags2(uint8_t value)
	{
		write(IrqFlags2::__address, value, 8);
	}
	
	/* Get register IrqFlags2 */
	uint8_t getIrqFlags2()
	{
		return read8(IrqFlags2::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                         REG DioMapping1                                          *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/* REG DioMapping1:
	 */
	struct DioMapping1
	{
		static const uint16_t __address = 64;
		
		/* Bits Dio0Mapping: */
		/*
		 * Mapping of pins DIO0 to DIO5
		 *           See Table 18 for mapping in LoRa mode
		 *           See Table 29 for mapping in Continuous mode SeeTable 30 for mapping in Packet mode 
		 */
		struct Dio0Mapping
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00; // 2'd0
			static const uint8_t mask = 0b11000000; // [6,7]
		};
		/* Bits Dio1Mapping: */
		struct Dio1Mapping
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00; // 2'd0
			static const uint8_t mask = 0b00110000; // [4,5]
		};
		/* Bits Dio2Mapping: */
		struct Dio2Mapping
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00; // 2'd0
			static const uint8_t mask = 0b00001100; // [2,3]
		};
		/* Bits Dio3Mapping: */
		struct Dio3Mapping
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00; // 2'd0
			static const uint8_t mask = 0b00000011; // [0,1]
		};
	};
	
	/* Set register DioMapping1 */
	void setDioMapping1(uint8_t value)
	{
		write(DioMapping1::__address, value, 8);
	}
	
	/* Get register DioMapping1 */
	uint8_t getDioMapping1()
	{
		return read8(DioMapping1::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                         REG DioMapping2                                          *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/* REG DioMapping2:
	 */
	struct DioMapping2
	{
		static const uint16_t __address = 65;
		
		/* Bits Dio4Mapping: */
		struct Dio4Mapping
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00; // 2'd0
			static const uint8_t mask = 0b11000000; // [6,7]
		};
		/* Bits Dio5Mapping: */
		struct Dio5Mapping
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00; // 2'd0
			static const uint8_t mask = 0b00110000; // [4,5]
		};
		/* Bits reserved_0: */
		/* Retain default value  */
		struct reserved_0
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b000; // 3'd0
			static const uint8_t mask = 0b00001110; // [1,2,3]
		};
		/* Bits MapPreambleDetect: */
		/*
		 * Allows the mapping of either Rssi Or PreambleDetect to the DIO pins,
		 *           as summarized on Table 29 and Table 30
		 *           0 = Rssi interrupt
		 *           1 = PreambleDetect interrupt 
		 */
		struct MapPreambleDetect
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00000001; // [0]
		};
	};
	
	/* Set register DioMapping2 */
	void setDioMapping2(uint8_t value)
	{
		write(DioMapping2::__address, value, 8);
	}
	
	/* Get register DioMapping2 */
	uint8_t getDioMapping2()
	{
		return read8(DioMapping2::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                           REG Version                                            *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG Version:
	 * Version code of the chip. Bits 7-4 give the full revision number; bits 3-0
	 *       give the metal mask revision number. 
	 */
	struct Version
	{
		static const uint16_t __address = 66;
		
		/* Bits Version: */
		struct Version_
		{
			/* Mode:r */
			static const uint8_t dflt = 0b00010010; // 8'h12
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register Version */
	void setVersion(uint8_t value)
	{
		write(Version::__address, value, 8);
	}
	
	/* Get register Version */
	uint8_t getVersion()
	{
		return read8(Version::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                            REG PllHop                                             *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG PllHop:
	 */
	struct PllHop
	{
		static const uint16_t __address = 68;
		
		/* Bits FastHopOn: */
		/*
		 * Bypasses the main state machine for a quick frequency hop. Writing RegFrfLsb
		 *           will trigger the frequency change.
		 *           0 = Frf is validated when FSTx or FSRx is requested
		 *           1 = Frf is validated triggered when RegFrfLsb is written 
		 */
		struct FastHopOn
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b10000000; // [7]
		};
		/* Bits reserved_0: */
		struct reserved_0
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0101101; // 7'h2d
			static const uint8_t mask = 0b01111111; // [0,1,2,3,4,5,6]
		};
	};
	
	/* Set register PllHop */
	void setPllHop(uint8_t value)
	{
		write(PllHop::__address, value, 8);
	}
	
	/* Get register PllHop */
	uint8_t getPllHop()
	{
		return read8(PllHop::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                             REG Tcxo                                              *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG Tcxo:
	 */
	struct Tcxo
	{
		static const uint16_t __address = 75;
		
		/* Bits reserved_0: */
		/* Retain default value  */
		struct reserved_0
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b000; // 3'd0
			static const uint8_t mask = 0b11100000; // [5,6,7]
		};
		/* Bits TcxoInputOn: */
		/*
		 * Controls the crystal oscillator
		 *           0 = Crystal Oscillator with external Crystal
		 *           1 = External clipped sine TCXO AC-connected to XTA pin 
		 */
		struct TcxoInputOn
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'd0
			static const uint8_t mask = 0b00010000; // [4]
		};
		/* Bits reserved_1: */
		/* Retain default value.  */
		struct reserved_1
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b1001; // 4'h9
			static const uint8_t mask = 0b00001111; // [0,1,2,3]
		};
	};
	
	/* Set register Tcxo */
	void setTcxo(uint8_t value)
	{
		write(Tcxo::__address, value, 8);
	}
	
	/* Get register Tcxo */
	uint8_t getTcxo()
	{
		return read8(Tcxo::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                            REG PaDac                                             *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/* REG PaDac:
	 */
	struct PaDac
	{
		static const uint16_t __address = 77;
		
		/* Bits reserved_0: */
		/* Retain default value  */
		struct reserved_0
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b10000; // 5'h10
			static const uint8_t mask = 0b11111000; // [3,4,5,6,7]
		};
		/* Bits PaDac: */
		/*
		 * Enables the +20dBm option on PA_BOOST pin
		 *           0x04 = Default value
		 *           0x07 = +20dBm on PA_BOOST when OutputPower=1111 
		 */
		struct PaDac_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b100; // 3'h4
			static const uint8_t mask = 0b00000111; // [0,1,2]
		};
	};
	
	/* Set register PaDac */
	void setPaDac(uint8_t value)
	{
		write(PaDac::__address, value, 8);
	}
	
	/* Get register PaDac */
	uint8_t getPaDac()
	{
		return read8(PaDac::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG FormerTemp                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG FormerTemp:
	 * Temperature saved during the latest IQ (RSSI and Image) calibration.
	 *       Same format as TempValue in RegTemp. 
	 */
	struct FormerTemp
	{
		static const uint16_t __address = 91;
		
		/* Bits FormerTemp: */
		struct FormerTemp_
		{
			/* Mode:rw */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register FormerTemp */
	void setFormerTemp(uint8_t value)
	{
		write(FormerTemp::__address, value, 8);
	}
	
	/* Get register FormerTemp */
	uint8_t getFormerTemp()
	{
		return read8(FormerTemp::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                         REG BitrateFrac                                          *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/* REG BitrateFrac:
	 */
	struct BitrateFrac
	{
		static const uint16_t __address = 93;
		
		/* Bits unused_0: */
		struct unused_0
		{
			/* Mode:r */
			static const uint8_t dflt = 0b0000; // 4'h0
			static const uint8_t mask = 0b11110000; // [4,5,6,7]
		};
		/* Bits BitRateFrac: */
		/*
		 * Fractional part of the bit rate divider (Only valid for FSK) If BitRateFrac> 0 then:
		 *           BitRate = 
		 */
		struct BitRateFrac
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0000; // 4'h0
			static const uint8_t mask = 0b00001111; // [0,1,2,3]
		};
	};
	
	/* Set register BitrateFrac */
	void setBitrateFrac(uint8_t value)
	{
		write(BitrateFrac::__address, value, 8);
	}
	
	/* Get register BitrateFrac */
	uint8_t getBitrateFrac()
	{
		return read8(BitrateFrac::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                            REG AgcRef                                             *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG AgcRef:
	 */
	struct AgcRef
	{
		static const uint16_t __address = 97;
		
		/* Bits unused_0: */
		struct unused_0
		{
			/* Mode:r */
			static const uint8_t mask = 0b11000000; // [6,7]
		};
		/* Bits AgcReferenceLevel: */
		/*
		 * Sets the floor reference for all AGC thresholds: AGC Reference[dBm]=
		 *           -174dBm+10*log(2*RxBw)+SNR+AgcReferenceLevel SNR = 8dB, fixed value 
		 */
		struct AgcReferenceLevel
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b011001; // 6'h19
			static const uint8_t mask = 0b00111111; // [0,1,2,3,4,5]
		};
	};
	
	/* Set register AgcRef */
	void setAgcRef(uint8_t value)
	{
		write(AgcRef::__address, value, 8);
	}
	
	/* Get register AgcRef */
	uint8_t getAgcRef()
	{
		return read8(AgcRef::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG AgcThresh1                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG AgcThresh1:
	 */
	struct AgcThresh1
	{
		static const uint16_t __address = 98;
		
		/* Bits unused_0: */
		struct unused_0
		{
			/* Mode:r */
			static const uint8_t mask = 0b11100000; // [5,6,7]
		};
		/* Bits AgcStep1: */
		/* Defines the 1st AGC Threshold  */
		struct AgcStep1
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b01100; // 5'hc
			static const uint8_t mask = 0b00011111; // [0,1,2,3,4]
		};
	};
	
	/* Set register AgcThresh1 */
	void setAgcThresh1(uint8_t value)
	{
		write(AgcThresh1::__address, value, 8);
	}
	
	/* Get register AgcThresh1 */
	uint8_t getAgcThresh1()
	{
		return read8(AgcThresh1::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG AgcThresh2                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG AgcThresh2:
	 */
	struct AgcThresh2
	{
		static const uint16_t __address = 99;
		
		/* Bits AgcStep2: */
		/* Defines the 2nd AGC Threshold:  */
		struct AgcStep2
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0100; // 4'h4
			static const uint8_t mask = 0b11110000; // [4,5,6,7]
		};
		/* Bits AgcStep3: */
		/* Defines the 3rd AGC Threshold:  */
		struct AgcStep3
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b1011; // 4'hb
			static const uint8_t mask = 0b00001111; // [0,1,2,3]
		};
	};
	
	/* Set register AgcThresh2 */
	void setAgcThresh2(uint8_t value)
	{
		write(AgcThresh2::__address, value, 8);
	}
	
	/* Get register AgcThresh2 */
	uint8_t getAgcThresh2()
	{
		return read8(AgcThresh2::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG AgcThresh3                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG AgcThresh3:
	 */
	struct AgcThresh3
	{
		static const uint16_t __address = 100;
		
		/* Bits AgcStep4: */
		/* Defines the 4th AGC Threshold:  */
		struct AgcStep4
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b1100; // 4'hc
			static const uint8_t mask = 0b11110000; // [4,5,6,7]
		};
		/* Bits AgcStep5: */
		/* Defines the 5th AGC Threshold:  */
		struct AgcStep5
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b1100; // 4'hc
			static const uint8_t mask = 0b00001111; // [0,1,2,3]
		};
	};
	
	/* Set register AgcThresh3 */
	void setAgcThresh3(uint8_t value)
	{
		write(AgcThresh3::__address, value, 8);
	}
	
	/* Get register AgcThresh3 */
	uint8_t getAgcThresh3()
	{
		return read8(AgcThresh3::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                            REG PllLf                                             *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/* REG PllLf:
	 */
	struct PllLf
	{
		static const uint16_t __address = 112;
		
		/* Bits PllBandwidth: */
		/* Controls the PLL bandwidth  */
		struct PllBandwidth
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b11; // 2'd3
			static const uint8_t mask = 0b11000000; // [6,7]
			static const uint8_t BW75k = 0b00; // 75 kHz
			static const uint8_t BW150k = 0b01; // 150 kHz
			static const uint8_t BW225k = 0b10; // 225 kHz
			static const uint8_t BW300k = 0b11; // 300 kHz
		};
		/* Bits reserved_0: */
		/* Retain default value  */
		struct reserved_0
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b010000; // 6'h10
			static const uint8_t mask = 0b00111111; // [0,1,2,3,4,5]
		};
	};
	
	/* Set register PllLf */
	void setPllLf(uint8_t value)
	{
		write(PllLf::__address, value, 8);
	}
	
	/* Get register PllLf */
	uint8_t getPllLf()
	{
		return read8(PllLf::__address, 8);
	}
	
};
