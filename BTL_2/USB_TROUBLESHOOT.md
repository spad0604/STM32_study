# USB HID Mouse Troubleshooting Guide

## Vấn đề hiện tại
Bạn đã kết nối đúng:
- PA11 -> USB D-
- PA12 -> USB D+

Nhưng máy tính chưa nhận diện được USB HID Mouse.

## Checklist để kiểm tra

### 1. Kiểm tra phần cứng
- ✅ PA11 kết nối với D- của USB
- ✅ PA12 kết nối với D+ của USB
- ❓ Có kết nối GND chung giữa STM32 và máy tính không?
- ❓ Có điện trở pull-up trên D+ (1.5kΩ) không? (USB Full Speed cần)
- ❓ Nguồn 3.3V ổn định không?

### 2. Kiểm tra clock configuration
- Clock đã được cấu hình đúng cho USB (48MHz) chưa?
- HSE 25MHz đã hoạt động chưa?
- PLL đã tạo ra 120MHz system clock chưa?

### 3. Kiểm tra USB configuration trong STM32CubeMX
- USB_OTG_FS đã được enable chưa?
- Mode đã set là "Device_Only" chưa?
- USB_DEVICE middleware đã được enable chưa?
- Class đã chọn là "Human Interface Device Class (HID)" chưa?

### 4. Debugging steps

#### Step 1: Kiểm tra USB enumeration
Thêm code để kiểm tra trạng thái USB:

```c
// Trong main loop, thêm:
if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
    // USB đã được nhận diện
    // Có thể toggle LED để báo hiệu
} else {
    // USB chưa được nhận diện
}
```

#### Step 2: Kiểm tra Windows Device Manager
1. Mở Device Manager
2. Kết nối STM32
3. Xem có thiết bị mới xuất hiện không:
   - Nếu xuất hiện "Unknown Device" -> Vấn đề về descriptor
   - Nếu xuất hiện "HID-compliant mouse" -> Thành công
   - Nếu không có gì -> Vấn đề về phần cứng/clock

#### Step 3: Sử dụng USB analyzer
Nếu có sẵn, dùng USB analyzer hoặc logic analyzer để:
- Kiểm tra USB differential signals
- Xem có USB reset/enumeration packets không

### 5. Các vấn đề thường gặp

#### Vấn đề 1: Clock không đúng
```c
// Kiểm tra trong SystemClock_Config():
// Phải có PLL với output 48MHz cho USB
RCC_OscInitStruct.PLL.PLLQ = 5; // 240MHz / 5 = 48MHz
```

#### Vấn đề 2: USB pins không được config đúng
```c
// PA11 và PA12 phải được config là USB_OTG_FS
// Không được set manual trong GPIO_Init
```

#### Vấn đề 3: Delay không đủ
```c
// Trong main(), sau MX_USB_DEVICE_Init():
HAL_Delay(2000); // Tăng delay lên 2 giây
```

#### Vấn đề 4: Report format không đúng
```c
// Buffer phải là 4 bytes: [buttons, x, y, wheel]
uint8_t buffer[4] = {0};
buffer[0] = 0;    // Buttons (0 = không nhấn)
buffer[1] = dx;   // X movement
buffer[2] = dy;   // Y movement  
buffer[3] = 0;    // Wheel
```

### 6. Test code đơn giản
Thử code test này để kiểm tra USB HID cơ bản:

```c
// Trong main loop:
static uint32_t last_time = 0;
if(HAL_GetTick() - last_time > 1000) { // Mỗi 1 giây
    last_time = HAL_GetTick();
    
    // Gửi movement nhỏ để test
    uint8_t test_buffer[4] = {0, 1, 0, 0}; // Di chuyển 1 pixel về phải
    USBD_HID_SendReport(&hUsbDeviceFS, test_buffer, 4);
}
```

### 7. Nếu vẫn không được
1. Thử reset lại STM32CubeMX project
2. Tạo project mới chỉ với USB HID
3. Kiểm tra lại connections với multimeter
4. Thử board khác nếu có
5. Kiểm tra USB cable (một số cable chỉ có power, không có data)

## Kết luận
Vấn đề thường là:
1. Clock configuration (90% trường hợp)
2. Hardware connections (5% trường hợp)  
3. USB descriptor issues (3% trường hợp)
4. Code logic (2% trường hợp) 