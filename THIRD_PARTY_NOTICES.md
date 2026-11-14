# Third-Party Notices

This T-Watch Ultra Arduino project uses the following third-party code and libraries.

# 1. DJI Osmo GPS Controller Demo

Portions of this project are derived from or inspired by the  
**Osmo GPS Controller Demo** published by SZ DJI Technology Co., Ltd.

The demo is offered under:

### DJI EULA  
http://developer.dji.com/policies/eula/

### MIT License
```
The MIT License (MIT)
Copyright (c) 2025 SZ DJI Technology Co., Ltd.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
```

**Full MIT License text:**  
https://opensource.org/licenses/MIT

The MIT terms above apply only to portions originating from DJI's MIT-licensed demo,  
and **not** to DJI EULA–restricted components.


# 2. Arduino Dependencies (T-Watch Ultra)

The Arduino / LilyGoLib port (`arduino/DJI_Remote_T-Watch/`) depends on the
third-party libraries below. Exact working versions are pinned in
`WORKING_LIBRARIES.md`.

| Library | License | Source |
|---------|---------|--------|
| LilyGoLib | MIT | https://github.com/Xinyuan-LilyGO/LilyGoLib |
| SensorLib | MIT (Bosch driver code in `src/bosch/` under BSD-3-Clause) | https://github.com/lewisxhe/SensorLib |
| RadioLib | MIT | https://github.com/jgromes/RadioLib |
| LVGL (v9.4.0, bundled via LilyGoLib-ThirdParty) | MIT | https://github.com/lvgl/lvgl |
| NimBLE-Arduino | Apache-2.0 | https://github.com/h2zero/NimBLE-Arduino |
| TinyGPSPlus | LGPL-2.1 | https://github.com/mikalhart/TinyGPSPlus |
| Arduino-ESP32 core | LGPL-2.1 (some components Apache-2.0) | https://github.com/espressif/arduino-esp32 |
| NFC-RFAL-fork | STMicroelectronics software license (RFAL / STSW-ST25RFAL002) | https://github.com/lewisxhe/NFC-RFAL-fork |
| ST25R3916-fork | STMicroelectronics software license (RFAL / STSW-ST25RFAL002) | https://github.com/lewisxhe/ST25R3916-fork |

The NFC libraries are compile-time dependencies of LilyGoLib and are **not** used
by this project's functionality. They are based on STMicroelectronics' RFAL
(STSW-ST25RFAL002) and remain subject to STMicroelectronics' software license
terms — see the respective repositories.

**License texts:**
- MIT — https://opensource.org/licenses/MIT
- Apache 2.0 — https://www.apache.org/licenses/LICENSE-2.0
- LGPL 2.1 — https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
- BSD 3-Clause — https://opensource.org/licenses/BSD-3-Clause


# 3. Summary

The project code is MIT licensed. DJI demo code retains its applicable DJI EULA or MIT terms. Arduino libraries retain their respective licenses listed above.
