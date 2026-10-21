# Simulator chạy NGUYÊN code STM32 trong `src/`

Simulator biên dịch **toàn bộ project Keil của bạn** (`MDK-ARM/EIU_MCR.uvprojx`: `Core/Src/*.c`, `MyLib/*.c`, `App/*.c`)
thành một thư viện cho PC và chạy **không sửa một dòng nào**. Không có code driver nào được sinh ra thay thế.

Chỉ phần "phần cứng" được giả lập (PC không có chân GPIO, ADC, timer):
- Thay cho các file `stm32f1xx_hal_*.c` của ST: `sim/fw_host2/sim_core.c` (GPIO, TIM, ADC+DMA, Flash, SysTick, ngắt).
- Header CMSIS/HAL **thật** của bạn (`Drivers/`) được dùng; asm của ARM được vô hiệu hoá bằng `sim_prelude.h`.
- Thanh ghi ngoại vi được ánh xạ đúng địa chỉ thật (TIM2 `0x40000000`…, Flash `0x08000000`), nên
  `motor.c` ghi `TIM2->CCR3/CCR4`, `servo.c` ghi `TIM3->CCR3`, `flash.c` đọc con trỏ `0x0801F800` đều chạy y như trên chip.

## Chạy
```
python -m pip install pygame-ce ziglang      # ziglang = trình biên dịch C, không cần cài gcc
python sim/simulator.py --firmware <đường dẫn tới thư mục src>
```
Nếu thư mục `src/` nằm cạnh `sim/` thì không cần `--firmware`. Kiểm tra không cần cửa sổ: thêm `--selftest`.
Chỉ chạy được trên Windows và Linux (macOS không cho ánh xạ địa chỉ thấp).

## Những gì chạy bằng code của bạn
1. `main()` của bạn chạy thật: `HAL_Init`, `SystemClock_Config`, `MX_*_Init`, `HAL_*_MspInit`.
2. `main.c` hiện **comment** `robot_init(...)`, `HAL_TIM_Base_Start_IT(&htim1)` và chưa gọi `loop()`.
   Simulator tự gọi đúng các dòng đó rồi lặp `loop(&MCR_robot)`. Khi bạn bỏ comment và đưa `loop()` vào `while(1)`,
   simulator tự phát hiện và chạy `main()` nguyên bản.
3. Ngắt: `SysTick_Handler` (1 ms) và `TIM1_UP_IRQHandler` (chu kỳ tính từ PSC/ARR bạn đặt, hiện ≈ 6.37 ms) trong
   `stm32f1xx_it.c` được gọi thật → `HAL_TIM_PeriodElapsedCallback` → `timer_handle`.
4. Khi bấm Run, simulator làm đúng quy trình của bạn: DIP=000, SW1 (học đen), SW2 (học trắng), sang DIP=100
   (`run`), bấm SW3. `v_compare` được tính bởi `update_black/update_white` của bạn.
5. Đầu ra đọc từ thanh ghi: PWM = `TIM2->CCR3/CCR4` / ARR, chiều = PB1/PB4, phanh = PA10/PA11,
   lái = `TIM3->CCR3` (µs) − 1472, 10 µs ≈ 1 đơn vị `handle()`.
   Nên `calib_servo` (DIP=010) thực sự làm lệch bánh lái trong mô phỏng.

## Giả định (chỉnh bằng tham số)
- Cảm biến đen = 3300, trắng = 300 (ADC 12 bit); ngưỡng do code của bạn tính `(đen+trắng)/2 + calib_weight`.
- Một lần `loop()` tốn `--loop-us` (mặc định 50 µs) thời gian CPU; mỗi lệnh HAL tốn 0.5 µs.
- Chân phanh chỉ có tác dụng khi PWM = 0 (`--brake-always` để phanh cả khi PWM ≠ 0).
- Tâm cơ khí servo = 1472 µs (`--servo-center`).
- Chưa mô phỏng: siêu âm / EXTI (DIP mode 6, 5), encoder.
- `check_line.c`, `control.c`, `run.c` không nằm trong project Keil nên không được build (giống Keil).
- Thông số xe: `--car-length --car-width --wheelbase --sensor-ahead --sensor-pitch --line-width --mm-per-px --max-speed --servo-gain --brake-ms`.

Giao diện (vẽ map, Start/Goal/Heading, PNG, kích thước xe) giữ nguyên như trước.
