import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time
import numpy as np

PORT = "COM7"
BAUD = 115200
MAX_POINTS = 500
WINDOW_SIZE = 10


CAL_WEIGHTS = np.array([0, 0, 191, 225, 291, 291, 326, 574])
CAL_FREQS   = np.array([92.137, 92.363, 93.720, 92.956, 92.350, 93.332, 93.232, 94.481])

_coeffs = np.polyfit(CAL_FREQS, CAL_WEIGHTS, 3)
freq_to_weight = np.poly1d(_coeffs)

print(f"Cubic model: weight = {_coeffs[0]:.4f}*f³ + {_coeffs[1]:.4f}*f² + {_coeffs[2]:.4f}*f + {_coeffs[3]:.2f}")

data = deque(maxlen=MAX_POINTS)
data_raw = deque(maxlen=MAX_POINTS)
times = deque(maxlen=MAX_POINTS)
start_time = time.time()

# Warm-up: รอให้ค่านิ่ง 3 วินาทีแรก
WARMUP_TIME = 3.0

ser = serial.Serial(PORT, BAUD, timeout=0.05)
time.sleep(2)
ser.reset_input_buffer()

fig, ax = plt.subplots(figsize=(14, 6))
line_filtered, = ax.plot([], [], "b-", linewidth=1.5, label="Filtered")
line_raw, = ax.plot([], [], "gray", linewidth=0.5, alpha=0.3, label="Raw")

ax.set_ylim(20, 100)
ax.set_xlim(0, WINDOW_SIZE)
ax.set_xlabel("Time (s)", fontsize=12)
ax.set_ylabel("Frequency (Hz)", fontsize=12)
ax.set_title("Realtime Weight Monitor (Cubic Model)", fontsize=14, fontweight="bold")
ax.grid(True, alpha=0.3, linestyle=":", linewidth=0.5)
ax.legend(loc="upper right", fontsize=10)

info_text = ax.text(0.02, 0.95, "", transform=ax.transAxes, fontsize=14,
                     verticalalignment="top", fontfamily="monospace",
                     bbox=dict(boxstyle="round,pad=0.4", facecolor="white",
                               edgecolor="gray", alpha=0.9))


def median_filter(arr, k=5):
    n = len(arr)
    out = np.copy(arr)
    half = k // 2
    for i in range(half, n - half):
        out[i] = np.median(arr[i - half:i + half + 1])
    return out


def update(frame):
    while ser.in_waiting:
        try:
            line_data = ser.readline().decode("utf-8", errors="ignore").strip()
            if line_data:
                freq = float(line_data)
                if 1 < freq < 50000:
                    now = time.time() - start_time
                    if now > WARMUP_TIME:  # ข้าม warmup
                        data_raw.append(freq)
                        times.append(now)
        except:
            pass

    # Filter
    if len(data_raw) > 5:
        raw_array = np.array(list(data_raw))
        filtered = median_filter(raw_array, k=5)
        if len(filtered) >= 11:
            kernel = np.ones(11) / 11
            filtered = np.convolve(filtered, kernel, mode='same')
        data.clear()
        data.extend(filtered)

    if len(times) > 1:
        line_raw.set_data(list(times), list(data_raw))
        line_filtered.set_data(list(times), list(data))

        current_t = times[-1]
        if current_t > WINDOW_SIZE:
            ax.set_xlim(current_t - WINDOW_SIZE, current_t)

        # Auto-scale Y
        if len(data_raw) > 2:
            all_vals = list(data_raw)
            ymin = min(all_vals) - 2
            ymax = max(all_vals) + 2
            ax.set_ylim(ymin, ymax)

        # คำนวณน้ำหนักจาก Cubic model
        latest_freq = data[-1] if len(data) > 0 else 0
        est_weight = float(freq_to_weight(latest_freq))
        est_weight = max(0, est_weight)  # ไม่ให้ติดลบ

        # เฉลี่ย 20 ค่าล่าสุดเพื่อให้นิ่ง
        if len(data) >= 20:
            recent = np.array(list(data)[-20:])
            avg_freq = float(np.mean(recent))
            avg_weight = max(0, float(freq_to_weight(avg_freq)))
        else:
            avg_freq = latest_freq
            avg_weight = est_weight

        info_text.set_text(
            f"Freq:    {latest_freq:>7.3f} Hz\n"
            f"Avg:     {avg_freq:>7.3f} Hz\n"
            f"Weight:  {avg_weight:>7.1f} g"
        )

    return line_filtered, line_raw, info_text


ani = animation.FuncAnimation(fig, update, interval=30, blit=False,
                               cache_frame_data=False)

plt.tight_layout()
plt.show()
ser.close()
