# IWR6843AOPEVM & ESP32-S3 USB Data Architecture

This document comprehensively outlines the hardware and software architecture enabling communication between the ESP32-S3 (acting as a USB Host) and the Texas Instruments IWR6843AOPEVM radar module. It also documents historical flaws discovered in the codebase and the engineering reasoning/sources used to resolve them.

---

## 1. Hardware Layer: USB Enumeration & Power Delivery

### The Connection Mechanism
The system connects via USB using the **Silicon Labs CP2105** Dual UART-to-USB Bridge located on the IWR6843AOPEVM (or MMWAVEICBOOST carrier board). 
When connected to the ESP32-S3, the `usb_manager` initializes the ESP-IDF USB Host stack, which discovers the CP2105 and mounts two Virtual COM ports via the CDC-ACM driver:
1. **CLI Port** (Command Line Interface): Operates at 115,200 baud. Used strictly for initializing the radar with configuration files.
2. **DATA Port**: Operates at 921,600 baud. Exclusively streams high-speed binary frame packets out of the radar's L3 memory once the sensor starts.

### The Enumeration & VBUS Diode Issue (Hardware Fix)
**The Problem:** Initial setups often encounter the error `ENUM: Bad transfer status 1: CHECK_FULL_DEV_DESC`. The CP2105 physically refuses to communicate its USB descriptors to the ESP32-S3.
**The Cause:** According to the **Silicon Labs CP2105 Datasheet**, the chip sits in a suspended state until it detects `5V` on its `VREGIN` / `VBUS` pin. Standard development boards (like the **ESP32-S3-DevKitC-1**) are wired as USB Devices. Looking at the **Espressif Schematics**, the DevKit employs a reverse-current protection Schottky diode on the USB port, which physically blocks the internal `5V` rail from leaving the board and reaching the CP2105. Even with a USB-OTG adapter triggering host mode, the physical diode stops power delivery.
**The Fix:** A physical splice of the USB cable's `VBUS` (Red) wire is required. By wiring the IWR6843-side of the `VBUS` wire directly to the unprotected `5V` breakout pin on the ESP32-S3 DevKit header, the CP2105 detects voltage, wakes its internal regulator, and successfully completes USB enumeration.

---

## 2. Configuration Layer

Before the radar can output data, it must ingest a configuration. This is handled by `send_config_task` in `iwr6843.c`.
The configuration profile (loaded via `vital_signs_cfg.h`) contains dozens of commands. The TI CLI firmware expects these exactly as formatted. 
* **Whitespace & Comments:** The driver reads the string line-by-line, dynamically strips trailing whitespaces/tabs, and ignores lines starting with `%`. 
* **Source of Truth:** This implementation strictly follows the **TI mmWave SDK User's Guide (v4.x)** rules for sending CFG commands sequentially and blocking until an `ACK` ("Done") or `NACK` ("Error") is received before advancing.

---

## 3. Data Streaming & Parsing Layer

Once `sensorStart` is echoed via the CLI port, the Radar begins blindly vomiting telemetry through the DATA port. 
The USB driver (`data_rx_cb`) offloads these unpredictable bursts into a FreeRTOS Stream Buffer (`xStreamBuffer`). The standalone `parser_task` drains this buffer to reconstitute standard mmWave Frames.

### Frame Anatomy (per TI mmWave SDK EDMA/UART spec)
Every single data transmission strictly follows this structure:
1. **Magic Word**: `0x02 0x01 0x04 0x03 0x06 0x05 0x08 0x07` (8 Bytes)
2. **Frame Header**: Contains total length, frame IDs, and the number of TLVs within the chunk. (32 Bytes remainder)
3. **TLVs (Type-Length-Value)**: Payload chunks formatted dynamically (Target Track list, Point Cloud, Vital Signs, etc.).

---

## 4. Software Vulnerabilities Resolved

Three major flaws in the binary parsing loop were identified and resolved to prevent `LoadProhibited` RTOS panics and optimize CPU usage.

### Flaw A: Integer Overflow in Bounds Checking
**The Vulnerability:** The parser checked for payload bounds using `if (off + hdr.length > payload_len)`. If a corrupted UART byte sent an impossibly large `hdr.length` (e.g., `0xFFFFFFE0`), adding the `off` variable wrapped the unsigned 32-bit integer over its maximum limit, returning it to `0`. The safety check evaluated to false, leading to monumental buffer overruns.
**The Fix:** Reconfigured using subtractive logic: `if (hdr.length > payload_len - off)`.
**Source:** CERT C Secure Coding Standard (INT30-C) & CWE-190.

### Flaw B: RTOS Scheduler Thrashing (1-Byte Sliding Window)
**The Vulnerability:** Searching for the 8-Byte Magic Word originally involved pulling *1 byte* from the RTOS stream buffer, evaluating it, shifting a local memory array, and requesting another byte. At 921,600 baud, this spawned thousands of redundant blocking `xStreamBufferReceive` system calls, starving the other threads.
**The Fix:** Instead of requesting bytes sequentially, the parser waits firmly for just `0x02` (the first magic byte), and only then requests the remaining 7 bytes. If it fails, it instantly resets.
**Source:** FreeRTOS API Reference on `xStreamBufferReceive` context-switching limits.

### Flaw C (CORRECTED): Per-TLV alignment was wrong, *whole-packet* alignment is right
**Earlier (incorrect) claim:** That each TLV payload is padded up to a 4-byte boundary, so the parser must do `off += (hdr.length + 3) & ~3`. This was sourced from misread E2E forum threads and produced 1–3 byte drift per TLV whenever a payload length wasn't already aligned (notably `TARGET_INDEX`, which is exactly `numDetectedObj` bytes).
**Reality:** TLVs are packed back-to-back with **no inter-TLV padding**. The *only* padding is at the end of the whole packet, where the SDK rounds the total transmission up to the next multiple of 32 bytes; this is reflected in `totalPacketLen` and is implicit in our `remaining = totalPacketLen - 40` payload buffer (the trailing zero bytes are simply never read because the parser stops after `numTLVs` iterations).
**Sources verified:**
- TI's reference Python parser `parseFrame.py` (in this repo, line 175): `frameData = frameData[tlvLength:]` — raw advance, no alignment.
- TI Vital Signs / People Counting demo docs ("Understanding UART Data Output Format"): "The end of the packet is padded so that the total packet length is always a multiple of 32 Bytes."
- The 32-byte (not 4-byte) padding is also documented in `iwr_frame_header_t.totalPacketLen` in `include/iwr6843.h`.
**Current code:** `parse_tlvs` uses `off += hdr.length;` directly.

---

## 5. Schema Validation

The C Header files mapping the struct dimensions were strictly verified against the Python implementations (`parseTLVs.py`) found in the TI Resource Explorer labs tailored for the **Vital Signs with People Tracking Demo (SDK 4.x)**.

*   `IWR_TLV_VITAL_SIGNS (1040)` => Confirmed `136 bytes` (`2H33f`).
*   `IWR_TLV_TARGET_LIST_3D (1010)` => Confirmed `112 bytes` (`I27f`).
*   `IWR_TLV_COMPRESSED_POINTS (1020)` => Confirmed `20 byte` header (`5f`) and recursive `8 byte` objects (`2bh2H`).

The complete C codebase is now fully optimized to decode this high-speed telemetry cleanly without crashing, blocking, or failing basic memory safety norms.