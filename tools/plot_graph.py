import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from scipy.signal import find_peaks

# =============================================
# 1. โหลดข้อมูล
# =============================================
df = pd.read_csv("log_20260424_031734.csv")  # ← ใส่ชื่อไฟล์

df.columns = ["time", "freq"]
df["time_str"] = df["time"]

# แปลงเวลาเป็นวินาที
def time_to_sec(t):
    parts = t.split(":")
    h, m = int(parts[0]), int(parts[1])
    s = float(parts[2])
    return h * 3600 + m * 60 + s

df["sec"] = df["time"].apply(time_to_sec)
df["sec"] = df["sec"] - df["sec"].iloc[0]  # เริ่มจาก 0

freq = df["freq"].values
t = df["sec"].values

# =============================================
# 2. หา Baseline
# =============================================
baseline = np.median(freq[freq < 9.0])
print(f"Baseline: {baseline:.3f} kHz")

# =============================================
# 3. ตรวจจับ Peak (แต่ละครั้งที่แทง)
# =============================================
threshold = baseline + 0.5  # ถือว่ากดเมื่อสูงกว่า baseline 0.5 kHz

# หา peak
peaks, properties = find_peaks(freq, height=threshold + 2, distance=10, prominence=1)

print(f"\nพบ {len(peaks)} peaks:")
for i, p in enumerate(peaks):
    print(f"  Peak {i+1}: t={t[p]:.3f}s  freq={freq[p]:.3f} kHz")

# =============================================
# 4. แยก event แต่ละครั้งที่แทง
# =============================================
events = []
in_event = False
event_start = 0

for i in range(len(freq)):
    if not in_event and freq[i] > threshold:
        in_event = True
        event_start = i
    elif in_event and freq[i] < baseline + 0.1:
        # จบ event - ต้องนิ่งอย่างน้อย 5 ตัว
        count_below = 0
        for j in range(i, min(i + 10, len(freq))):
            if freq[j] < baseline + 0.1:
                count_below += 1
        if count_below >= 5:
            events.append({
                "start": event_start,
                "end": i,
                "t_start": t[event_start],
                "t_end": t[i],
                "peak_freq": np.max(freq[event_start:i]),
                "duration": t[i] - t[event_start],
                "data": freq[event_start:i],
                "time": t[event_start:i] - t[event_start]
            })
            in_event = False

print(f"\nพบ {len(events)} events (การแทง):")
for i, e in enumerate(events):
    print(f"  Event {i+1}: peak={e['peak_freq']:.3f} kHz  duration={e['duration']:.3f}s")

# =============================================
# 5. Fit Exponential Decay Model
# =============================================
def exp_decay(t, A, tau, f0):
    """f(t) = A * exp(-t/tau) + f0"""
    return A * np.exp(-t / tau) + f0

# รวมทุก event (ใช้ช่วงขาลงหลัง peak สูงสุด)
all_t_decay = []
all_f_decay = []

for e in events:
    data = e["data"]
    time_e = e["time"]
    
    # หาจุด peak สูงสุดใน event
    peak_idx = np.argmax(data)
    
    # เอาเฉพาะช่วงขาลง (หลัง peak ตัวสุดท้าย)
    # หา peak ตัวที่ 2 (ถ้ามี)
    local_peaks, _ = find_peaks(data, height=baseline + 2, distance=5)
    if len(local_peaks) > 1:
        start_decay = local_peaks[-1]  # เริ่มจาก peak สุดท้าย
    else:
        start_decay = peak_idx
    
    t_decay = time_e[start_decay:] - time_e[start_decay]
    f_decay = data[start_decay:]
    
    all_t_decay.extend(t_decay)
    all_f_decay.extend(f_decay)

all_t_decay = np.array(all_t_decay)
all_f_decay = np.array(all_f_decay)

# Fit
try:
    popt, pcov = curve_fit(exp_decay, all_t_decay, all_f_decay,
                           p0=[5.0, 0.3, baseline],
                           maxfev=10000)
    A, tau, f0 = popt
    print(f"\n=== Exponential Decay Model ===")
    print(f"f(t) = {A:.3f} * exp(-t / {tau:.3f}) + {f0:.3f}")
    print(f"A (amplitude): {A:.3f} kHz")
    print(f"tau (time constant): {tau:.3f} s")
    print(f"f0 (baseline): {f0:.3f} kHz")
except:
    print("Fit failed, ใช้ค่าเฉลี่ย")
    A, tau, f0 = 5.0, 0.3, baseline

# =============================================
# 6. สร้างโมเดล Peak → น้ำหนัก
# =============================================
peak_values = [e["peak_freq"] for e in events]
print(f"\n=== Peak Analysis ===")
print(f"Peak min: {min(peak_values):.3f} kHz")
print(f"Peak max: {max(peak_values):.3f} kHz")
print(f"Peak mean: {np.mean(peak_values):.3f} kHz")
print(f"Peak std: {np.std(peak_values):.3f} kHz")

# =============================================
# 7. Plot ทุกอย่าง
# =============================================
fig, axes = plt.subplots(2, 2, figsize=(14, 10))

# Plot 1: Raw data + events
ax1 = axes[0, 0]
ax1.plot(t, freq, "b-", linewidth=0.8, alpha=0.7)
ax1.axhline(y=baseline, color="r", linestyle="--", label=f"Baseline: {baseline:.2f} kHz")
ax1.axhline(y=threshold, color="orange", linestyle="--", alpha=0.5, label=f"Threshold: {threshold:.2f} kHz")
for i, e in enumerate(events):
    ax1.axvspan(e["t_start"], e["t_end"], alpha=0.2, color="green")
    ax1.plot(t[e["start"] + np.argmax(e["data"])], e["peak_freq"], "rv", markersize=10)
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Frequency (kHz)")
ax1.set_title("Raw Data + Detected Events")
ax1.legend()
ax1.grid(True, alpha=0.3)

# Plot 2: Overlay events
ax2 = axes[0, 1]
colors = plt.cm.tab10(np.linspace(0, 1, len(events)))
for i, e in enumerate(events):
    ax2.plot(e["time"], e["data"], color=colors[i], label=f"Event {i+1} (peak={e['peak_freq']:.1f})")
ax2.set_xlabel("Time from event start (s)")
ax2.set_ylabel("Frequency (kHz)")
ax2.set_title("All Events Overlaid")
ax2.legend(fontsize=8)
ax2.grid(True, alpha=0.3)

# Plot 3: Decay fit
ax3 = axes[1, 0]
ax3.scatter(all_t_decay, all_f_decay, s=5, alpha=0.3, label="Data")
t_fit = np.linspace(0, max(all_t_decay), 200)
f_fit = exp_decay(t_fit, A, tau, f0)
ax3.plot(t_fit, f_fit, "r-", linewidth=2,
         label=f"Fit: {A:.2f}*exp(-t/{tau:.3f})+{f0:.2f}")
ax3.set_xlabel("Time from peak (s)")
ax3.set_ylabel("Frequency (kHz)")
ax3.set_title("Exponential Decay Fit")
ax3.legend()
ax3.grid(True, alpha=0.3)

# Plot 4: Peak histogram
ax4 = axes[1, 1]
ax4.hist(peak_values, bins=10, edgecolor="black", alpha=0.7)
ax4.axvline(x=np.mean(peak_values), color="r", linestyle="--",
            label=f"Mean: {np.mean(peak_values):.2f} kHz")
ax4.set_xlabel("Peak Frequency (kHz)")
ax4.set_ylabel("Count")
ax4.set_title("Peak Distribution")
ax4.legend()
ax4.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("analysis_result.png", dpi=150)
plt.show()

# =============================================
# 8. สรุปโมเดลสำหรับใส่ Arduino
# =============================================
print("\n" + "=" * 50)
print("  สรุปค่าสำหรับใส่ Arduino")
print("=" * 50)
print(f"const float BASELINE = {f0 * 1000:.1f};     // Hz")
print(f"const float THRESHOLD = {threshold * 1000:.1f};  // Hz เริ่มตรวจจับ")
print(f"const float DECAY_A = {A * 1000:.1f};       // Hz amplitude")
print(f"const float DECAY_TAU = {tau:.4f};       // seconds")
print(f"const float PEAK_MEAN = {np.mean(peak_values) * 1000:.1f};  // Hz")
print(f"const float PEAK_STD = {np.std(peak_values) * 1000:.1f};   // Hz")