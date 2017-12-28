/*
 * name:        SX1276_77_78_79
 * description: 137 MHz to 1020 MHz Low Power Long Range Transceiver featuring the LoRa (TM) long range modem
 * manuf:       Semtech
 * version:     0.1
 * url:         http://www.semtech.com/images/datasheet/sx1276_77_78_79.pdf
 * date:        2016-08-01
 * author       https://chisl.io/
 * file:        SX1276_77_78_79_LORA.hpp
 */

#include <cinttypes>

/* Derive from class SX1276_77_78_79_LORA_Base and implement the read and write functions! */

/* SX1276_77_78_79: 137 MHz to 1020 MHz Low Power Long Range Transceiver featuring the LoRa (TM) long range modem */
class SX1276_77_78_79_LORA_Base
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
	 * LoRaTM base-band FIFO data input/output.
	 * FIFO is cleared an not accessible when device is in SLEEP mode
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
		 * 0 = FSK/OOK Mode
		 * 1 = LoRaTM Mode
		 * This bit can be modified only in Sleep mode. A write operation on other device modes is ignored.
		 */
		struct LongRangeMode
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'h0
			static const uint8_t mask = 0b10000000; // [7]
		};
		/* Bits AccessSharedReg: */
		/*
		 * This bit operates when device is in Lora mode; if set it allows access to FSK registers page located in address space (0x0D:0x3F) while in LoRa mode
		 * 0 = Access LoRa registers page 0x0D: 0x3F
		 * 1 = Access FSK registers page (in mode LoRa) 0x0D: 0x3F
		 */
		struct AccessSharedReg
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'h0
			static const uint8_t mask = 0b01000000; // [6]
		};
		/* Bits reserved_0: */
		/* reserved  */
		struct reserved_0
		{
			/* Mode:r */
			static const uint8_t dflt = 0b00; // 2'h0
			static const uint8_t mask = 0b00110000; // [4,5]
		};
		/* Bits LowFrequencyModeOn: */
		/* Access Low Frequency Mode registers  */
		struct LowFrequencyModeOn
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b1; // 1'h1
			static const uint8_t mask = 0b00001000; // [3]
			static const uint8_t HIGH_FREQUENCY_MODE = 0b0; // High Frequency Mode (access to HF test registers)
			static const uint8_t LOW_FREQUENCY_MODE = 0b1; // Low Frequency Mode (access to LF test registers) §
		};
		/* Bits Mode: */
		/* Device modes  */
		struct Mode
		{
			/* Mode:rwt */
			static const uint8_t dflt = 0b001; // 3'h1
			static const uint8_t mask = 0b00000111; // [0,1,2]
			static const uint8_t SLEEP = 0b00; // 
			static const uint8_t STDBY = 0b01; // 
			static const uint8_t FSTX = 0b10; // Frequency synthesis TX
			static const uint8_t TX = 0b11; // Transmit
			static const uint8_t FSRX = 0b100; // Frequency synthesis RX
			static const uint8_t RXCONTINUOUS = 0b101; // Receive continuous
			static const uint8_t RXSINGLE = 0b110; // receive single
			static const uint8_t CAD = 0b111; // Channel activity detection
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
	 *                                              REG Fr                                               *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG Fr:
	 * RF carrier frequency
	 * Resolution is 61.035 Hz if F(XOSC) = 32 MHz.
	 * Default value is 0x6c8000 = 434 MHz.
	 * Register values must be modified only when device is in SLEEP or STAND-BY mode.
	 */
	struct Fr
	{
		static const uint16_t __address = 6;
		
		/* Bits Fr: */
		struct Fr_
		{
			/* Mode:rw */
			static const uint32_t dflt = 0b011011001000000000000000; // 24'h6c8000
			static const uint32_t mask = 0b111111111111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23]
		};
	};
	
	/* Set register Fr */
	void setFr(uint32_t value)
	{
		write(Fr::__address, value, 24);
	}
	
	/* Get register Fr */
	uint32_t getFr()
	{
		return read32(Fr::__address, 24);
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
			static const uint8_t dflt = 0b0; // 1'h0
			static const uint8_t mask = 0b10000000; // [7]
			static const uint8_t RFO = 0b0; // RFO pin. Output power is limited to +14 dBm.
			static const uint8_t PA_BOOST = 0b1; // PA_BOOST pin. Output power is limited to +20 dBm
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
		 * Pout=Pmax-(15-OutputPower) if PaSelect = 0 (RFO pin)
		 * Pout=17-(15-OutputPower)   if PaSelect = 1 (PA_BOOST pin)
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
		/* unused  */
		struct unused_0
		{
			/* Mode:r */
			static const uint8_t dflt = 0b000; // 3'd0
			static const uint8_t mask = 0b11100000; // [5,6,7]
		};
		/* Bits reserved_1: */
		struct reserved_1
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'h0
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
			static const uint8_t dflt = 0b00; // 2'h0
			static const uint8_t mask = 0b11000000; // [6,7]
		};
		/* Bits OcpOn: */
		/*
		 * Enables overload current protection (OCP) for PA:
		 * 0 = OCP disabled
		 * 1 = OCP enabled
		 */
		struct OcpOn
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b1; // 1'h1
			static const uint8_t mask = 0b00100000; // [5]
		};
		/* Bits OcpTrim: */
		/*
		 * Trimming of OCP current:
		 * Imax = 45+5*OcpTrim [mA] if OcpTrim <= 15 (120 mA)
		 * Imax = -30+10*OcpTrim [mA] if 15 < OcpTrim <= 27 (130 to 240 mA)
		 * Imax = 240mA for higher settings
		 * Default Imax = 100mA
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
		 * LNA gain setting:
		 * b000 not used
		 * b111 not used
		 */
		struct LnaGain
		{
			/* Mode:rwx */
			static const uint8_t dflt = 0b001; // 3'b1
			static const uint8_t mask = 0b11100000; // [5,6,7]
			static const uint8_t G1 = 0b01; // maximum gain
			static const uint8_t G2 = 0b10; // 
			static const uint8_t G3 = 0b11; // 
			static const uint8_t G4 = 0b100; // 
			static const uint8_t G5 = 0b101; // 
			static const uint8_t G6 = 0b110; // minimum gain
		};
		/* Bits LnaBoostLf: */
		/*
		 * Low Frequency (RFI_LF) LNA current adjustment
		 * 00 = Default LNA current
		 * Other = Reserved
		 */
		struct LnaBoostLf
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00; // 2'b0
			static const uint8_t mask = 0b00011000; // [3,4]
		};
		/* Bits reserved_0: */
		struct reserved_0
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000100; // [2]
		};
		/* Bits LnaBoostHf: */
		/*
		 * High Frequency (RFI_HF) LNA current adjustment
		 * 0 = Default;     -- Default LNA current
		 * 1 = Boost_on; -- Boost on, 150% LNA current
		 */
		struct LnaBoostHf
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00; // 2'b0
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
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                         REG FifoAddrPtr                                          *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG FifoAddrPtr:
	 * SPI interface address pointer in FIFO data buffer.
	 */
	struct FifoAddrPtr
	{
		static const uint16_t __address = 13;
		
		/* Bits FifoAddrPtr: */
		struct FifoAddrPtr_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00000000; // 8'h0
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register FifoAddrPtr */
	void setFifoAddrPtr(uint8_t value)
	{
		write(FifoAddrPtr::__address, value, 8);
	}
	
	/* Get register FifoAddrPtr */
	uint8_t getFifoAddrPtr()
	{
		return read8(FifoAddrPtr::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                        REG FifoTxBaseAddr                                         *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG FifoTxBaseAddr:
	 * write base address in FIFO data buffer for TX modulator
	 */
	struct FifoTxBaseAddr
	{
		static const uint16_t __address = 14;
		
		/* Bits FifoTxBaseAddr: */
		struct FifoTxBaseAddr_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b10000000; // 8'h80
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register FifoTxBaseAddr */
	void setFifoTxBaseAddr(uint8_t value)
	{
		write(FifoTxBaseAddr::__address, value, 8);
	}
	
	/* Get register FifoTxBaseAddr */
	uint8_t getFifoTxBaseAddr()
	{
		return read8(FifoTxBaseAddr::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                        REG FifoRxBaseAddr                                         *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG FifoRxBaseAddr:
	 * read base address in FIFO data buffer for RX demodulator
	 */
	struct FifoRxBaseAddr
	{
		static const uint16_t __address = 15;
		
		/* Bits FifoRxBaseAddr: */
		struct FifoRxBaseAddr_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00000000; // 8'h0
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register FifoRxBaseAddr */
	void setFifoRxBaseAddr(uint8_t value)
	{
		write(FifoRxBaseAddr::__address, value, 8);
	}
	
	/* Get register FifoRxBaseAddr */
	uint8_t getFifoRxBaseAddr()
	{
		return read8(FifoRxBaseAddr::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                      REG FifoRxCurrentAddr                                       *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG FifoRxCurrentAddr:
	 * Start address (in data buffer) of last packet received
	 */
	struct FifoRxCurrentAddr
	{
		static const uint16_t __address = 16;
		
		/* Bits FifoRxCurrentAddr: */
		struct FifoRxCurrentAddr_
		{
			/* Mode:r */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register FifoRxCurrentAddr */
	void setFifoRxCurrentAddr(uint8_t value)
	{
		write(FifoRxCurrentAddr::__address, value, 8);
	}
	
	/* Get register FifoRxCurrentAddr */
	uint8_t getFifoRxCurrentAddr()
	{
		return read8(FifoRxCurrentAddr::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                         REG IrqFlagsMask                                          *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG IrqFlagsMask:
	 */
	struct IrqFlagsMask
	{
		static const uint16_t __address = 17;
		
		/* Bits RxTimeoutMask: */
		/* Timeout interrupt mask: setting this bit masks the corresponding IRQ in RegIrqFlags  */
		struct RxTimeoutMask
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b10000000; // [7]
		};
		/* Bits RxDoneMask: */
		/*
		 * Packet reception complete interrupt mask: setting this bit masks the
		 * corresponding IRQ in RegIrqFlags
		 */
		struct RxDoneMask
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b01000000; // [6]
		};
		/* Bits PayloadCrcErrorMask: */
		/*
		 * Payload CRC error interrupt mask: setting this bit masks the
		 * corresponding IRQ in RegIrqFlags
		 */
		struct PayloadCrcErrorMask
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00100000; // [5]
		};
		/* Bits ValidHeaderMask: */
		/*
		 * Valid header received in Rx mask: setting this bit masks the
		 * corresponding IRQ in RegIrqFlags
		 */
		struct ValidHeaderMask
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00010000; // [4]
		};
		/* Bits TxDoneMask: */
		/*
		 * FIFO Payload transmission complete interrupt mask: setting this bit masks
		 * the corresponding IRQ in RegIrqFlags
		 */
		struct TxDoneMask
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00001000; // [3]
		};
		/* Bits CadDoneMask: */
		/*
		 * CAD complete interrupt mask: setting this bit masks the corresponding
		 * IRQ in RegIrqFlags
		 */
		struct CadDoneMask
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000100; // [2]
		};
		/* Bits FhssChangeChannelMask: */
		/*
		 * FHSS change channel interrupt mask: setting this bit masks the
		 * corresponding IRQ in RegIrqFlags
		 */
		struct FhssChangeChannelMask
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000010; // [1]
		};
		/* Bits CadDetectedMask: */
		/*
		 * Cad Detected Interrupt Mask: setting this bit masks the corresponding
		 * IRQ in RegIrqFlags
		 */
		struct CadDetectedMask
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000001; // [0]
		};
	};
	
	/* Set register IrqFlagsMask */
	void setIrqFlagsMask(uint8_t value)
	{
		write(IrqFlagsMask::__address, value, 8);
	}
	
	/* Get register IrqFlagsMask */
	uint8_t getIrqFlagsMask()
	{
		return read8(IrqFlagsMask::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG IrqFlags                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG IrqFlags:
	 */
	struct IrqFlags
	{
		static const uint16_t __address = 18;
		
		/* Bits RxTimeout: */
		/* Timeout interrupt: writing a 1 clears the IRQ  */
		struct RxTimeout
		{
			/* Mode:rc */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b10000000; // [7]
		};
		/* Bits RxDone: */
		/* Packet reception complete interrupt: writing a 1 clears the IRQ  */
		struct RxDone
		{
			/* Mode:rc */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b01000000; // [6]
		};
		/* Bits PayloadCrcError: */
		/* Payload CRC error interrupt: writing a 1 clears the IRQ  */
		struct PayloadCrcError
		{
			/* Mode:rc */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00100000; // [5]
		};
		/* Bits ValidHeader: */
		/* Valid header received in Rx: writing a 1 clears the IRQ  */
		struct ValidHeader
		{
			/* Mode:rc */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00010000; // [4]
		};
		/* Bits TxDone: */
		/* FIFO Payload transmission complete interrupt: writing a 1 clears the IRQ  */
		struct TxDone
		{
			/* Mode:rc */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00001000; // [3]
		};
		/* Bits CadDone: */
		/* CAD complete: write to clear: writing a 1 clears the IRQ  */
		struct CadDone
		{
			/* Mode:rc */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000100; // [2]
		};
		/* Bits FhssChangeChannel: */
		/* FHSS change channel interrupt: writing a 1 clears the IRQ  */
		struct FhssChangeChannel
		{
			/* Mode:rc */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000010; // [1]
		};
		/* Bits CadDetected: */
		/* Valid Lora signal detected during CAD operation: writing a 1 clears the IRQ  */
		struct CadDetected
		{
			/* Mode:rc */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000001; // [0]
		};
	};
	
	/* Set register IrqFlags */
	void setIrqFlags(uint8_t value)
	{
		write(IrqFlags::__address, value, 8);
	}
	
	/* Get register IrqFlags */
	uint8_t getIrqFlags()
	{
		return read8(IrqFlags::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG RxNbBytes                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG RxNbBytes:
	 * Number of payload bytes of latest packet received
	 */
	struct RxNbBytes
	{
		static const uint16_t __address = 19;
		
		/* Bits RxNbBytes: */
		struct RxNbBytes_
		{
			/* Mode:r */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register RxNbBytes */
	void setRxNbBytes(uint8_t value)
	{
		write(RxNbBytes::__address, value, 8);
	}
	
	/* Get register RxNbBytes */
	uint8_t getRxNbBytes()
	{
		return read8(RxNbBytes::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                       REG RxHeaderCntValue                                        *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG RxHeaderCntValue:
	 * Number of valid headers received since last transition into Rx mode.
	 * Header and packet counters are reseted in Sleep mode.
	 */
	struct RxHeaderCntValue
	{
		static const uint16_t __address = 20;
		
		/* Bits RxHeaderCntValue: */
		struct RxHeaderCntValue_
		{
			/* Mode:r */
			static const uint16_t mask = 0b1111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
		};
	};
	
	/* Set register RxHeaderCntValue */
	void setRxHeaderCntValue(uint16_t value)
	{
		write(RxHeaderCntValue::__address, value, 16);
	}
	
	/* Get register RxHeaderCntValue */
	uint16_t getRxHeaderCntValue()
	{
		return read16(RxHeaderCntValue::__address, 16);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                       REG RxPacketCntValue                                        *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG RxPacketCntValue:
	 * Number of valid packets received since last transition into Rx mode.
	 * Header and packet counters are reseted in Sleep mode.
	 */
	struct RxPacketCntValue
	{
		static const uint16_t __address = 22;
		
		/* Bits RxPacketCntValue: */
		struct RxPacketCntValue_
		{
			/* Mode:rc */
			static const uint16_t mask = 0b1111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
		};
	};
	
	/* Set register RxPacketCntValue */
	void setRxPacketCntValue(uint16_t value)
	{
		write(RxPacketCntValue::__address, value, 16);
	}
	
	/* Get register RxPacketCntValue */
	uint16_t getRxPacketCntValue()
	{
		return read16(RxPacketCntValue::__address, 16);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG ModemStat                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/* REG ModemStat:
	 */
	struct ModemStat
	{
		static const uint16_t __address = 24;
		
		/* Bits RxCodingRate: */
		/* Coding rate of last header received  */
		struct RxCodingRate
		{
			static const uint8_t mask = 0b11100000; // [5,6,7]
		};
		/* Bits ModemClear: */
		/* Modem clear  */
		struct ModemClear
		{
			static const uint8_t dflt = 0b1; // 1'b1
			static const uint8_t mask = 0b00010000; // [4]
		};
		/* Bits HeaderInfoValid: */
		/* Header info valid  */
		struct HeaderInfoValid
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00001000; // [3]
		};
		/* Bits RxOngoing: */
		/* RX on-going  */
		struct RxOngoing
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000100; // [2]
		};
		/* Bits SignalSynchronized: */
		/* Signal synchronized  */
		struct SignalSynchronized
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000010; // [1]
		};
		/* Bits SignalDetected: */
		/* Signal detected  */
		struct SignalDetected
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000001; // [0]
		};
	};
	
	/* Set register ModemStat */
	void setModemStat(uint8_t value)
	{
		write(ModemStat::__address, value, 8);
	}
	
	/* Get register ModemStat */
	uint8_t getModemStat()
	{
		return read8(ModemStat::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                         REG PktSnrValue                                          *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG PktSnrValue:
	 * Estimation of SNR on last packet received.In two's compliment format mutiplied by 4.
	 * SNR[dB] = PacketSnr[twos complement-] / 4
	 */
	struct PktSnrValue
	{
		static const uint16_t __address = 25;
		
		/* Bits PktSnrValue: */
		struct PktSnrValue_
		{
			/* Mode:r */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register PktSnrValue */
	void setPktSnrValue(uint8_t value)
	{
		write(PktSnrValue::__address, value, 8);
	}
	
	/* Get register PktSnrValue */
	uint8_t getPktSnrValue()
	{
		return read8(PktSnrValue::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                         REG PktRssiValue                                          *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG PktRssiValue:
	 * RSSI of the latest packet received (dBm):
	 * RSSI[dBm] = -157 + Rssi (using HF output port, SNR >= 0) or
	 * RSSI[dBm] = -164 + Rssi (using LF output port, SNR >= 0)
	 * (see section 5.5.5 for details)
	 */
	struct PktRssiValue
	{
		static const uint16_t __address = 26;
		
		/* Bits PktRssiValue: */
		struct PktRssiValue_
		{
			/* Mode:r */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register PktRssiValue */
	void setPktRssiValue(uint8_t value)
	{
		write(PktRssiValue::__address, value, 8);
	}
	
	/* Get register PktRssiValue */
	uint8_t getPktRssiValue()
	{
		return read8(PktRssiValue::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG RssiValue                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG RssiValue:
	 * Current RSSI value (dBm)
	 * RSSI[dBm] = -157 + Rssi (using HF output port) or
	 * RSSI[dBm] = -164 + Rssi (using LF output port)
	 * (see section 5.5.5 for details)
	 */
	struct RssiValue
	{
		static const uint16_t __address = 27;
		
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
	 *                                          REG HopChannel                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG HopChannel:
	 */
	struct HopChannel
	{
		static const uint16_t __address = 28;
		
		/* Bits PllTimeout: */
		/*
		 * PLL failed to lock while attempting a TX/RX/CAD operation 1 = PLL did not lock
		 * 0 = PLL did lock
		 */
		struct PllTimeout
		{
			/* Mode:r */
			static const uint8_t mask = 0b10000000; // [7]
		};
		/* Bits CrcOnPayload: */
		/*
		 * CRC Information extracted from the received packet header (Explicit header mode only)
		 * 0 = Header indicates CRC off
		 * 1 = Header indicates CRC on
		 */
		struct CrcOnPayload
		{
			/* Mode:r */
			static const uint8_t mask = 0b01000000; // [6]
		};
		/* Bits FhssPresentChannel: */
		/* Current value of frequency hopping channel in use.  */
		struct FhssPresentChannel
		{
			/* Mode:r */
			static const uint8_t mask = 0b00111111; // [0,1,2,3,4,5]
		};
	};
	
	/* Set register HopChannel */
	void setHopChannel(uint8_t value)
	{
		write(HopChannel::__address, value, 8);
	}
	
	/* Get register HopChannel */
	uint8_t getHopChannel()
	{
		return read8(HopChannel::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                         REG ModemConfig1                                          *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG ModemConfig1:
	 */
	struct ModemConfig1
	{
		static const uint16_t __address = 29;
		
		/* Bits BW: */
		/*
		 * Signal bandwidth:
		 * In the lower band (169MHz), signal bandwidths 8&9 are not supported)
		 * other values = reserved
		 */
		struct BW
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0111; // 4'b111
			static const uint8_t mask = 0b11110000; // [4,5,6,7]
			static const uint8_t BW_7_8 = 0b000; // 
			static const uint8_t BW_10_4 = 0b001; // 
			static const uint8_t BW_15_6 = 0b010; // 
			static const uint8_t BW_20_8 = 0b011; // 
			static const uint8_t BW_31_25 = 0b100; // 
			static const uint8_t BW_41_7 = 0b101; // 
			static const uint8_t BW_62_5 = 0b110; // 
			static const uint8_t BW_125 = 0b111; // 
			static const uint8_t BW_250 = 0b1000; // 
			static const uint8_t BW_500 = 0b1001; // 
		};
		/* Bits CodingRate: */
		/*
		 * Error coding rate
		 * In implicit header mode should be set on receiver to determine
		 * expected coding rate. See 4.1.1.3
		 * All other values = reserved
		 */
		struct CodingRate
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b001; // 3'b1
			static const uint8_t mask = 0b00001110; // [1,2,3]
			static const uint8_t CR_4_5 = 0b01; // 
			static const uint8_t CR_4_6 = 0b10; // 
			static const uint8_t CR_4_7 = 0b11; // 
			static const uint8_t CR_4_8 = 0b100; // 
		};
		/* Bits ImplicitHeaderModeOn: */
		/*
		 * 0 = Explicit Header mode
		 * 1 = Implicit Header mode
		 */
		struct ImplicitHeaderModeOn
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000001; // [0]
		};
	};
	
	/* Set register ModemConfig1 */
	void setModemConfig1(uint8_t value)
	{
		write(ModemConfig1::__address, value, 8);
	}
	
	/* Get register ModemConfig1 */
	uint8_t getModemConfig1()
	{
		return read8(ModemConfig1::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                         REG ModemConfig2                                          *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG ModemConfig2:
	 */
	struct ModemConfig2
	{
		static const uint16_t __address = 30;
		
		/* Bits SpreadingFactor: */
		/* SF rate (expressed as a base-2 logarithm)  */
		struct SpreadingFactor
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0111; // 4'h7
			static const uint8_t mask = 0b11110000; // [4,5,6,7]
			static const uint8_t SF_64 = 6; // 
			static const uint8_t SF_128 = 7; // 
			static const uint8_t SF_256 = 8; // 
			static const uint8_t SF_512 = 9; // 
			static const uint8_t SF_1024 = 10; // 
			static const uint8_t SF_2048 = 11; // 
			static const uint8_t SF_4096 = 12; // 
		};
		/* Bits TxContinuousMode: */
		/*
		 * 0 = normal mode, a single packet is sent
		 * 1 = continuous mode, send multiple packets across the FIFO (used for spectral analysis)
		 */
		struct TxContinuousMode
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00001000; // [3]
		};
		/* Bits RxPayloadCrcOn: */
		/*
		 * Enable CRC generation and check on payload: 0 = CRC disable
		 * 1 = CRC enable
		 * If CRC is needed, RxPayloadCrcOn should be set:
		 * - in Implicit header mode: on Tx and Rx side
		 * - in Explicit header mode: on the Tx side alone (recovered from the header in Rx side)
		 */
		struct RxPayloadCrcOn
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000100; // [2]
		};
		/* Bits SymbTimeout: */
		/* RX Time-Out MSB (bits 9:8)  */
		struct SymbTimeout
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00; // 2'b0
			static const uint8_t mask = 0b00000011; // [0,1]
		};
	};
	
	/* Set register ModemConfig2 */
	void setModemConfig2(uint8_t value)
	{
		write(ModemConfig2::__address, value, 8);
	}
	
	/* Get register ModemConfig2 */
	uint8_t getModemConfig2()
	{
		return read8(ModemConfig2::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                        REG SymbTimeoutLsb                                         *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG SymbTimeoutLsb:
	 * RX Time-Out LSB
	 * RX operation time-out value expressed as number of symbols:
	 * TimeOut = SymbTimeout / Ts
	 */
	struct SymbTimeoutLsb
	{
		static const uint16_t __address = 31;
		
		/* Bits SymbTimeoutLsb: */
		struct SymbTimeoutLsb_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b01100100; // 8'h64
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register SymbTimeoutLsb */
	void setSymbTimeoutLsb(uint8_t value)
	{
		write(SymbTimeoutLsb::__address, value, 8);
	}
	
	/* Get register SymbTimeoutLsb */
	uint8_t getSymbTimeoutLsb()
	{
		return read8(SymbTimeoutLsb::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG Preamble                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG Preamble:
	 * Preamble length MSB, = PreambleLength + 4.25 Symbols See 4.1.1 for more details.
	 */
	struct Preamble
	{
		static const uint16_t __address = 32;
		
		/* Bits Preamble: */
		struct Preamble_
		{
			/* Mode:rw */
			static const uint16_t dflt = 0b0000000000001000; // 16'h8
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
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                        REG PayloadLength                                         *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG PayloadLength:
	 * Payload length in bytes. The register needs to be set in implicit header
	 * mode for the expected packet length. A 0 value is not permitted
	 */
	struct PayloadLength
	{
		static const uint16_t __address = 34;
		
		/* Bits PayloadLength: */
		struct PayloadLength_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00000001; // 8'h1
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
	 *                                       REG MaxPayloadLength                                        *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG MaxPayloadLength:
	 * Maximum payload length; if header payload length exceeds value a header
	 * CRC error is generated. Allows filtering of packet with a bad size.
	 */
	struct MaxPayloadLength
	{
		static const uint16_t __address = 35;
		
		/* Bits MaxPayloadLength: */
		struct MaxPayloadLength_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b11111111; // 8'hff
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register MaxPayloadLength */
	void setMaxPayloadLength(uint8_t value)
	{
		write(MaxPayloadLength::__address, value, 8);
	}
	
	/* Get register MaxPayloadLength */
	uint8_t getMaxPayloadLength()
	{
		return read8(MaxPayloadLength::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG HopPeriod                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG HopPeriod:
	 * Symbol periods between frequency hops. (0 = disabled).
	 * 1st hop always happen after the 1st header symbol
	 */
	struct HopPeriod
	{
		static const uint16_t __address = 36;
		
		/* Bits HopPeriod: */
		struct HopPeriod_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00000000; // 8'h0
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register HopPeriod */
	void setHopPeriod(uint8_t value)
	{
		write(HopPeriod::__address, value, 8);
	}
	
	/* Get register HopPeriod */
	uint8_t getHopPeriod()
	{
		return read8(HopPeriod::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                        REG FifoRxByteAddr                                         *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG FifoRxByteAddr:
	 * Current value of RX databuffer pointer
	 * (address of last byte written by Lora receiver)
	 */
	struct FifoRxByteAddr
	{
		static const uint16_t __address = 37;
		
		/* Bits FifoRxByteAddr: */
		struct FifoRxByteAddr_
		{
			/* Mode:r */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register FifoRxByteAddr */
	void setFifoRxByteAddr(uint8_t value)
	{
		write(FifoRxByteAddr::__address, value, 8);
	}
	
	/* Get register FifoRxByteAddr */
	uint8_t getFifoRxByteAddr()
	{
		return read8(FifoRxByteAddr::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                         REG ModemConfig3                                          *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG ModemConfig3:
	 */
	struct ModemConfig3
	{
		static const uint16_t __address = 38;
		
		/* Bits Unused_0: */
		/* None  */
		struct Unused_0
		{
			/* Mode:r */
			static const uint8_t dflt = 0b0000; // 4'h0
			static const uint8_t mask = 0b11110000; // [4,5,6,7]
		};
		/* Bits LowDataRateOptimize: */
		/*
		 * 0 = Disabled
		 * 1 = Enabled; mandated for when the symbol length exceeds 16ms
		 */
		struct LowDataRateOptimize
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00001000; // [3]
		};
		/* Bits AgcAutoOn: */
		/*
		 * 0 = LNA gain set by register LnaGain
		 * 1 = LNA gain set by the internal AGC loop
		 */
		struct AgcAutoOn
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000100; // [2]
		};
		/* Bits Reserved_1: */
		struct Reserved_1
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00; // 2'b0
			static const uint8_t mask = 0b00000011; // [0,1]
		};
	};
	
	/* Set register ModemConfig3 */
	void setModemConfig3(uint8_t value)
	{
		write(ModemConfig3::__address, value, 8);
	}
	
	/* Get register ModemConfig3 */
	uint8_t getModemConfig3()
	{
		return read8(ModemConfig3::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                        REG PpmCorrection                                         *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG PpmCorrection:
	 * Data rate offset value, used in conjunction with AFC
	 */
	struct PpmCorrection
	{
		static const uint16_t __address = 39;
		
		/* Bits PpmCorrection: */
		struct PpmCorrection_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00000000; // 8'h0
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register PpmCorrection */
	void setPpmCorrection(uint8_t value)
	{
		write(PpmCorrection::__address, value, 8);
	}
	
	/* Get register PpmCorrection */
	uint8_t getPpmCorrection()
	{
		return read8(PpmCorrection::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                             REG Fei                                              *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/* REG Fei:
	 */
	struct Fei
	{
		static const uint16_t __address = 40;
		
		/* Bits Reserved_0: */
		struct Reserved_0
		{
			/* Mode:r */
			static const uint32_t mask = 0b111100000000000000000000; // [20,21,22,23]
		};
		/* Bits FreqError: */
		/* Estimated frequency error from modem MSB of RF Frequency Error  */
		struct FreqError
		{
			/* Mode:r */
			static const uint32_t dflt = 0b00000000000000000000; // 20'd0
			static const uint32_t mask = 0b000011111111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19]
		};
	};
	
	/* Set register Fei */
	void setFei(uint32_t value)
	{
		write(Fei::__address, value, 24);
	}
	
	/* Get register Fei */
	uint32_t getFei()
	{
		return read32(Fei::__address, 24);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                         REG RssiWideband                                          *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG RssiWideband:
	 * Wideband RSSI measurement used to locally generate a random number
	 */
	struct RssiWideband
	{
		static const uint16_t __address = 44;
		
		/* Bits RssiWideband: */
		struct RssiWideband_
		{
			/* Mode:r */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register RssiWideband */
	void setRssiWideband(uint8_t value)
	{
		write(RssiWideband::__address, value, 8);
	}
	
	/* Get register RssiWideband */
	uint8_t getRssiWideband()
	{
		return read8(RssiWideband::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                        REG DetectOptimize                                         *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG DetectOptimize:
	 */
	struct DetectOptimize
	{
		static const uint16_t __address = 49;
		
		/* Bits Reserved_0: */
		/* Reserved  */
		struct Reserved_0
		{
			/* Mode:r */
			static const uint8_t dflt = 0b11000; // 5'h18
			static const uint8_t mask = 0b11111000; // [3,4,5,6,7]
		};
		/* Bits DetectionOptimize: */
		/* LoRa Detection Optimize  */
		struct DetectionOptimize
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b011; // 3'h3
			static const uint8_t mask = 0b00000111; // [0,1,2]
			static const uint8_t SF7_12 = 0x3; // SF7 to SF12
			static const uint8_t SF6 = 0x5; // 
		};
	};
	
	/* Set register DetectOptimize */
	void setDetectOptimize(uint8_t value)
	{
		write(DetectOptimize::__address, value, 8);
	}
	
	/* Get register DetectOptimize */
	uint8_t getDetectOptimize()
	{
		return read8(DetectOptimize::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG InvertIQ                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/* REG InvertIQ:
	 */
	struct InvertIQ
	{
		static const uint16_t __address = 51;
		
		/* Bits Reserved_0: */
		/* Reserved  */
		struct Reserved_0
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b10000000; // [7]
		};
		/* Bits InvertIQ: */
		/*
		 * Invert the LoRa I and Q signals 0 = normal mode
		 * 1 = I and Q signals are inverted
		 */
		struct InvertIQ_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b01000000; // [6]
		};
		/* Bits Reserved_1: */
		/* Reserved  */
		struct Reserved_1
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b100111; // 6'h27
			static const uint8_t mask = 0b00111111; // [0,1,2,3,4,5]
		};
	};
	
	/* Set register InvertIQ */
	void setInvertIQ(uint8_t value)
	{
		write(InvertIQ::__address, value, 8);
	}
	
	/* Get register InvertIQ */
	uint8_t getInvertIQ()
	{
		return read8(InvertIQ::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                      REG DetectionThreshold                                       *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG DetectionThreshold:
	 * LoRa detection threshold
	 */
	struct DetectionThreshold
	{
		static const uint16_t __address = 55;
		
		/* Bits Threshold: */
		struct Threshold
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00001010; // 8'ha
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
			static const uint8_t SF7_12 = 0xa; // SF7 to SF12
			static const uint8_t SF6 = 0xc; // 
		};
	};
	
	/* Set register DetectionThreshold */
	void setDetectionThreshold(uint8_t value)
	{
		write(DetectionThreshold::__address, value, 8);
	}
	
	/* Get register DetectionThreshold */
	uint8_t getDetectionThreshold()
	{
		return read8(DetectionThreshold::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG SyncWord                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG SyncWord:
	 * LoRa Sync Word
	 * Value 0x34 is reserved for LoRaWAN networks
	 */
	struct SyncWord
	{
		static const uint16_t __address = 57;
		
		/* Bits SyncWord: */
		struct SyncWord_
		{
			/* Mode:rw */
			static const uint8_t dflt = 0b00010010; // 8'h12
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register SyncWord */
	void setSyncWord(uint8_t value)
	{
		write(SyncWord::__address, value, 8);
	}
	
	/* Get register SyncWord */
	uint8_t getSyncWord()
	{
		return read8(SyncWord::__address, 8);
	}
	
};
