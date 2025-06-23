#!/usr/bin/env python3
"""
UART Mouse Controller
Nhận dữ liệu mouse từ STM32 qua UART và điều khiển con trỏ chuột trên PC

Cách dùng:
1. Kết nối STM32 UART1 với USB-UART module
2. Cài đặt: pip install pyautogui pyserial
3. Chạy: python uart_mouse.py
4. Chọn COM port đúng
"""

import serial
import pyautogui
import time
import sys
from threading import Thread
import tkinter as tk
from tkinter import messagebox, ttk

class UARTMouse:
    def __init__(self):
        self.serial_port = None
        self.running = False
        self.sensitivity = 1.0
        
        # Tắt pyautogui fail-safe
        pyautogui.FAILSAFE = True
        pyautogui.PAUSE = 0.001  # Giảm delay
        
    def list_serial_ports(self):
        """Liệt kê các COM ports có sẵn"""
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    
    def connect_serial(self, port, baudrate=115200):
        """Kết nối đến serial port"""
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            print(f"✅ Đã kết nối {port} @ {baudrate} baud")
            return True
        except Exception as e:
            print(f"❌ Lỗi kết nối {port}: {e}")
            return False
    
    def parse_mouse_data(self, line):
        """Parse dữ liệu: MOUSE:dx,dy"""
        try:
            if line.startswith("MOUSE:"):
                data = line.replace("MOUSE:", "").strip().split(",")
                if len(data) >= 2:
                    dx = int(data[0]) * self.sensitivity
                    dy = int(data[1]) * self.sensitivity
                    return dx, dy
        except Exception as e:
            print(f"Parse error: {e}")
        return None, None
    
    def mouse_thread(self):
        """Thread đọc UART và điều khiển chuột"""
        print("🖱️  Bắt đầu điều khiển chuột...")
        print("📌 Nhấn Ctrl+C để dừng")
        
        while self.running:
            try:
                if self.serial_port and self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    
                    if line:
                        print(f"📥 Nhận: {line}")
                        
                        dx, dy = self.parse_mouse_data(line)
                        if dx is not None and dy is not None:
                            # Di chuyển chuột
                            pyautogui.moveRel(dx, dy)
                            print(f"🖱️  Di chuyển: ({dx}, {dy})")
                
                time.sleep(0.01)  # 100Hz polling
                
            except Exception as e:
                print(f"❌ Lỗi: {e}")
                time.sleep(0.1)
    
    def start_gui(self):
        """GUI để chọn COM port và điều khiển"""
        root = tk.Tk()
        root.title("UART Mouse Controller")
        root.geometry("400x300")
        
        # COM Port selection
        tk.Label(root, text="Chọn COM Port:", font=("Arial", 12)).pack(pady=10)
        
        port_var = tk.StringVar()
        port_combo = ttk.Combobox(root, textvariable=port_var, width=15)
        port_combo['values'] = self.list_serial_ports()
        port_combo.pack(pady=5)
        
        # Sensitivity
        tk.Label(root, text="Độ nhạy:", font=("Arial", 10)).pack(pady=(20,5))
        sensitivity_var = tk.DoubleVar(value=1.0)
        sensitivity_scale = tk.Scale(root, from_=0.1, to=5.0, resolution=0.1, 
                                   orient=tk.HORIZONTAL, variable=sensitivity_var)
        sensitivity_scale.pack(pady=5)
        
        # Status
        status_var = tk.StringVar(value="Chưa kết nối")
        status_label = tk.Label(root, textvariable=status_var, fg="red")
        status_label.pack(pady=10)
        
        def start_mouse():
            port = port_var.get()
            if not port:
                messagebox.showerror("Lỗi", "Vui lòng chọn COM port!")
                return
                
            self.sensitivity = sensitivity_var.get()
            
            if self.connect_serial(port):
                self.running = True
                status_var.set(f"✅ Đang chạy trên {port}")
                status_label.config(fg="green")
                
                # Bắt đầu thread
                mouse_thread = Thread(target=self.mouse_thread, daemon=True)
                mouse_thread.start()
                
                start_btn.config(state="disabled")
                stop_btn.config(state="normal")
            else:
                messagebox.showerror("Lỗi", f"Không thể kết nối {port}")
        
        def stop_mouse():
            self.running = False
            if self.serial_port:
                self.serial_port.close()
            status_var.set("⏹️  Đã dừng")
            status_label.config(fg="orange")
            start_btn.config(state="normal")
            stop_btn.config(state="disabled")
        
        # Buttons
        button_frame = tk.Frame(root)
        button_frame.pack(pady=20)
        
        start_btn = tk.Button(button_frame, text="▶️ Bắt đầu", 
                             command=start_mouse, bg="lightgreen")
        start_btn.pack(side=tk.LEFT, padx=10)
        
        stop_btn = tk.Button(button_frame, text="⏹️ Dừng", 
                            command=stop_mouse, bg="lightcoral", state="disabled")
        stop_btn.pack(side=tk.LEFT, padx=10)
        
        # Instructions
        instructions = """
📋 Hướng dẫn:
1. Kết nối STM32 UART1 với USB-UART module
2. Module USB vào máy tính  
3. Chọn COM port tương ứng
4. Điều chỉnh độ nhạy phù hợp
5. Nhấn "Bắt đầu" để điều khiển chuột

📊 Format data: MOUSE:dx,dy
Ví dụ: MOUSE:5,-3 = phải 5, lên 3 pixels
        """
        
        text_widget = tk.Text(root, height=8, width=50, wrap=tk.WORD)
        text_widget.pack(pady=10, padx=20, fill=tk.BOTH, expand=True)
        text_widget.insert(tk.END, instructions)
        text_widget.config(state=tk.DISABLED)
        
        root.protocol("WM_DELETE_WINDOW", lambda: (stop_mouse(), root.destroy()))
        root.mainloop()

def main():
    """Main function"""
    print("🖱️  UART Mouse Controller")
    print("=" * 40)
    
    mouse_controller = UARTMouse()
    
    # Kiểm tra command line arguments
    if len(sys.argv) > 1:
        # CLI mode
        port = sys.argv[1]
        baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
        
        if mouse_controller.connect_serial(port, baudrate):
            mouse_controller.running = True
            try:
                mouse_controller.mouse_thread()
            except KeyboardInterrupt:
                print("\n🛑 Dừng bởi người dùng")
        else:
            print(f"❌ Không thể kết nối {port}")
    else:
        # GUI mode
        try:
            mouse_controller.start_gui()
        except ImportError:
            print("❌ Không thể khởi động GUI. Cài đặt tkinter.")
            print("💡 Sử dụng CLI: python uart_mouse.py COM3 115200")

if __name__ == "__main__":
    main() 