import serial
import time
import csv
import os

PORT = "COM5"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

filename = "training_data.csv"

# สร้างไฟล์ใหม่ถ้ายังไม่มี
if not os.path.exists(filename):
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["weight_g", "peak_freq", "area_under_curve",
                         "rise_time", "decay_time", "freq_at_0.1s",
                         "freq_at_0.2s", "freq_at_0.3s", "baseline"])

print("=" * 50)
print("  ML Training Data Collector")
print("=" * 50)
print()

while True:
    # ถามน้ำหนัก
    weight = input("\nใส่น้ำหนัก (g) หรือ 'q' เพื่อจบ: ").strip()
    if weight.lower() == 'q':
        break
    
    try:
        weight_g = float(weight)
    except:
        print("ใส่ตัวเลข!")
        continue
    
    reps = int(input("จะกดกี่ครั้ง? "))
    
    for rep in range(reps):
        print(f"\n  ครั้งที่ {rep+1}/{reps}: กดเลย! (รอตรวจจับ...)")
        
        # อ่าน baseline
        readings = []
        for _ in range(20):
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                try:
                    readings.append(float(line))
                except:
                    pass
        
        if not readings:
            print("  ไม่มีสัญญาณ!")
            continue
            
        baseline = sum(readings) / len(readings)
        threshold = baseline + 500  # +0.5 kHz
        
        # รอให้กด
        event_data = []
        event_times = []
        detecting = False
        start_time = None
        
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            try:
                freq = float(line)
            except:
                continue
            
            if not detecting:
                if freq > threshold:
                    detecting = True
                    start_time = time.time()
                    event_data = [freq]
                    event_times = [0]
                    print(f"  ตรวจจับ! freq={freq:.0f}")
            else:
                elapsed = time.time() - start_time
                event_data.append(freq)
                event_times.append(elapsed)
                
                # จบ event: กลับ baseline + ผ่าน 0.3 วิ
                if freq < baseline + 100 and elapsed > 0.3:
                    break
                
                # timeout 3 วิ
                if elapsed > 3.0:
                    break
        
        if len(event_data) < 5:
            print("  event สั้นเกินไป ข้าม")
            continue
        
        # ★ Extract features
        import numpy as np
        
        data = np.array(event_data)
        times = np.array(event_times)
        
        peak_freq = np.max(data)
        peak_idx = np.argmax(data)
        
        # Area under curve (สูงกว่า baseline)
        area = np.trapz(data - baseline, times)
        
        # Rise time (baseline → peak)
        rise_time = times[peak_idx] if peak_idx > 0 else 0
        
        # Decay time (peak → กลับ baseline)
        decay_time = times[-1] - times[peak_idx]
        
        # Frequency at specific times
        def freq_at_time(t_target):
            for i in range(len(times) - 1):
                if times[i] <= t_target <= times[i + 1]:
                    ratio = (t_target - times[i]) / (times[i + 1] - times[i])
                    return data[i] + ratio * (data[i + 1] - data[i])
            return baseline
        
        f_01 = freq_at_time(0.1)
        f_02 = freq_at_time(0.2)
        f_03 = freq_at_time(0.3)
        
        # บันทึก
        with open(filename, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([weight_g, peak_freq, area, rise_time,
                           decay_time, f_01, f_02, f_03, baseline])
        
        print(f"  ✅ บันทึก: peak={peak_freq:.0f} area={area:.0f} "
              f"rise={rise_time:.3f}s decay={decay_time:.3f}s")

ser.close()
print(f"\nบันทึกเสร็จ: {filename}")