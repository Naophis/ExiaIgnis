# RP2350 ピンアサイン

## GPIO ピンマップ（機能別）

### モーター PWM

| GPIO | 機能 | 備考 |
|------|------|------|
| GPIO4 | M_PWM_L1 | 左モーター Ch1 |
| GPIO5 | M_PWM_L2 | 左モーター Ch2 |
| GPIO6 | M_PWM_R1 | 右モーター Ch1 |
| GPIO7 | M_PWM_R2 | 右モーター Ch2 |

### 吸引ファン

| GPIO | 機能 | 備考 |
|------|------|------|
| GPIO8 | suction_pwm1 | 吸引 PWM Ch1 |
| GPIO9 | suction_pwm3 | 吸引 PWM Ch3 |
| GPIO10 | suction_pwm3 | 吸引 PWM Ch3 |
| GPIO11 | suction_en | 吸引イネーブル |

### SPI1（共有バス: ジャイロ / エンコーダー / バッテリーADC）

| GPIO | 機能 | 備考 |
|------|------|------|
| GPIO12 | GYRO_MISO | MISO（全デバイス共有） |
| GPIO14 | GYRO_CLK | CLK（全デバイス共有） |
| GPIO15 | GYRO_MOSI | MOSI（全デバイス共有） |
| GPIO13 | GYRO_CS | CS: ASM330LHH ジャイロ (mode 3) |
| GPIO1 | ENC_CS_L | CS: AS5147P 左エンコーダー (mode 1) |
| GPIO17 | ENC_CS_R | CS: AS5147P 右エンコーダー (mode 1) |
| GPIO3 | Battery_CS | CS: ADS7042 バッテリーADC (mode 0) |

### センサー LED（発光）

| GPIO | 機能 | 備考 |
|------|------|------|
| GPIO16 | R90_LED | 右90° センサー LED |
| GPIO19 | R45_LED2 | 右45° センサー LED2 |
| GPIO20 | R45_LED1 | 右45° センサー LED1 |
| GPIO21 | L45_LED1 | 左45° センサー LED1 |
| GPIO24 | L45_LED2 | 左45° センサー LED2 |
| GPIO25 | L90_LED | 左90° センサー LED |

### センサー受光（ADC）

| GPIO | 機能 | ADC ch | 備考 |
|------|------|--------|------|
| GPIO26 | R90_SEN | ADC0 | 右90° 受光センサー |
| GPIO27 | R45_SEN | ADC1 | 右45° 受光センサー |
| GPIO28 | L45_SEN | ADC2 | 左45° 受光センサー |
| GPIO29 | L90_SEN | ADC3 | 左90° 受光センサー |

### LED ドライバー（I2C）

| GPIO | 機能 | 備考 |
|------|------|------|
| GPIO22 | LED_DRV_SDA | I2C SDA |
| GPIO23 | LED_DRV_SCL | I2C SCL |

### その他

| GPIO | 機能 | 備考 |
|------|------|------|
| GPIO0 | PSRAM_CE | PSRAM チップイネーブル |
| GPIO2 | SW1 | ユーザースイッチ |
| GPIO18 | buzzer | ブザー |

---

## 物理ピン一覧（ピン番号順）

| ピン# | NAME | GPIO | アサイン |
|-------|------|------|---------|
| 1 | IOVDD | — | — |
| 2 | — | GPIO0 | PSRAM_CE |
| 3 | — | GPIO1 | ENC_CS_L |
| 4 | — | GPIO2 | SW1 |
| 5 | — | GPIO3 | Battery_CS |
| 6 | DVDD | — | — |
| 7 | — | GPIO4 | M_PWM_L1 |
| 8 | — | GPIO5 | M_PWM_L2 |
| 9 | — | GPIO6 | M_PWM_R1 |
| 10 | — | GPIO7 | M_PWM_R2 |
| 11 | IOVDD | — | — |
| 12 | — | GPIO8 | suction_pwm1 |
| 13 | — | GPIO9 | suction_pwm2 |
| 14 | — | GPIO10 | suction_pwm3 |
| 15 | — | GPIO11 | suction_en |
| 16 | — | GPIO12 | GYRO_MISO (SPI1 MISO) |
| 17 | — | GPIO13 | GYRO_CS |
| 18 | — | GPIO14 | GYRO_CLK (SPI1 CLK) |
| 19 | — | GPIO15 | GYRO_MOSI (SPI1 MOSI) |
| 20 | IOVDD | — | — |
| 21 | XIN | — | 水晶振動子 IN |
| 22 | XOUT | — | 水晶振動子 OUT |
| 23 | DVDD | — | — |
| 24 | SWCLK | — | SWD クロック |
| 25 | SWDIO | — | SWD データ |
| 26 | RUN | — | リセット |
| 27 | — | GPIO16 | R90_LED |
| 28 | — | GPIO17 | ENC_CS_R |
| 29 | — | GPIO18 | buzzer |
| 30 | IOVDD | — | — |
| 31 | — | GPIO19 | R45_LED2 |
| 32 | — | GPIO20 | R45_LED1 |
| 33 | — | GPIO21 | L45_LED1 |
| 34 | — | GPIO22 | LED_DRV_SDA |
| 35 | — | GPIO23 | LED_DRV_SCL |
| 36 | — | GPIO24 | L45_LED2 |
| 37 | — | GPIO25 | L90_LED |
| 38 | IOVDD | — | — |
| 39 | DVDD | — | — |
| 40 | — | GPIO26 | R90_SEN (ADC0) |
| 41 | — | GPIO27 | R45_SEN (ADC1) |
| 42 | — | GPIO28 | L45_SEN (ADC2) |
| 43 | — | GPIO29 | L90_SEN (ADC3) |
| 44 | ADC_AVDD | — | ADC アナログ電源 |
| 45 | IOVDD | — | — |
| 46 | VREG_AVDD | — | 内部レギュレータ |
| 47 | VREG_PGND | — | 内部レギュレータ GND |
| 48 | VREG_LX | — | 内部レギュレータ SW |
| 49 | VREG_VIN | — | 内部レギュレータ入力 |
| 50 | VREG_FB | — | 内部レギュレータ FB |
| 51 | USB_DM | — | USB D- |
| 52 | USB_DP | — | USB D+ |
| 53 | USB_OTP_VDD | — | USB / OTP 電源 |
| 54 | QSPI_IOVDD | — | QSPI IO 電源 |
| 55 | QSPI_SD3 | — | QSPI データ3 |
| 56 | QSPI_SCLK | — | QSPI クロック |
| 57 | QSPI_SD0 | — | QSPI データ0 |
| 58 | QSPI_SD2 | — | QSPI データ2 |
| 59 | QSPI_SD1 | — | QSPI データ1 |
| 60 | QSPI_SS | — | QSPI チップセレクト（フラッシュ） |

---

## 備考

- **SPI1 共有バス**（MISO=GPIO12 / CLK=GPIO14 / MOSI=GPIO15）にジャイロ・左右エンコーダー・バッテリーADC を CS で切り替えて接続。CLAUDE.md 記載の構成と一致
- GPIO9 / GPIO10 が共に `suction_pwm3` — 同一信号を2ピンで駆動している可能性あり（要確認）
- GPIO18（buzzer）は未確定
