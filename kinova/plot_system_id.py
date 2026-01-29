"""
Plot system identification data for link6.
Reads system_id_link6_data.csv and generates visualization of:
- Spatial velocity (wx, wy, wz, vx, vy, vz)
- Spatial acceleration (dwx, dwy, dwz, dvx, dvy, dvz) with gravity compensation
- Commanded torque (tau6)

Run with:
    python plot_system_id.py
"""
import numpy as np
import matplotlib.pyplot as plt
import os


def load_data(filepath):
    """Load system ID data from CSV."""
    data = np.loadtxt(filepath, delimiter=',', skiprows=1)
    return {
        'time': data[:, 0],
        'wx': data[:, 1],
        'wy': data[:, 2],
        'wz': data[:, 3],
        'vx': data[:, 4],
        'vy': data[:, 5],
        'vz': data[:, 6],
        'dwx': data[:, 7],
        'dwy': data[:, 8],
        'dwz': data[:, 9],
        'dvx': data[:, 10],
        'dvy': data[:, 11],
        'dvz': data[:, 12],
        'tau6': data[:, 13],
    }


def plot_system_id_data(data, output_path):
    """Generate plots for system ID data."""
    t = data['time']
    
    fig, axes = plt.subplots(4, 4, figsize=(16, 12))
    
    # Row 1: Angular velocity (wx, wy, wz)
    axes[0, 0].plot(t, data['wx'], 'r-', linewidth=0.5)
    axes[0, 0].set_ylabel('wx (rad/s)')
    axes[0, 0].set_title('Angular Velocity X')
    axes[0, 0].grid(True, alpha=0.3)
    
    axes[0, 1].plot(t, data['wy'], 'g-', linewidth=0.5)
    axes[0, 1].set_ylabel('wy (rad/s)')
    axes[0, 1].set_title('Angular Velocity Y')
    axes[0, 1].grid(True, alpha=0.3)
    
    axes[0, 2].plot(t, data['wz'], 'b-', linewidth=0.5)
    axes[0, 2].set_ylabel('wz (rad/s)')
    axes[0, 2].set_title('Angular Velocity Z')
    axes[0, 2].grid(True, alpha=0.3)
    
    # Row 2: Linear velocity (vx, vy, vz)
    axes[1, 0].plot(t, data['vx'], 'r-', linewidth=0.5)
    axes[1, 0].set_ylabel('vx (m/s)')
    axes[1, 0].set_title('Linear Velocity X')
    axes[1, 0].grid(True, alpha=0.3)
    
    axes[1, 1].plot(t, data['vy'], 'g-', linewidth=0.5)
    axes[1, 1].set_ylabel('vy (m/s)')
    axes[1, 1].set_title('Linear Velocity Y')
    axes[1, 1].grid(True, alpha=0.3)
    
    axes[1, 2].plot(t, data['vz'], 'b-', linewidth=0.5)
    axes[1, 2].set_ylabel('vz (m/s)')
    axes[1, 2].set_title('Linear Velocity Z')
    axes[1, 2].grid(True, alpha=0.3)
    
    # Row 3: Angular acceleration (dwx, dwy, dwz)
    axes[2, 0].plot(t, data['dwx'], 'r-', linewidth=0.5)
    axes[2, 0].set_ylabel('dwx (rad/s²)')
    axes[2, 0].set_title('Angular Accel X')
    axes[2, 0].grid(True, alpha=0.3)
    
    axes[2, 1].plot(t, data['dwy'], 'g-', linewidth=0.5)
    axes[2, 1].set_ylabel('dwy (rad/s²)')
    axes[2, 1].set_title('Angular Accel Y')
    axes[2, 1].grid(True, alpha=0.3)
    
    axes[2, 2].plot(t, data['dwz'], 'b-', linewidth=0.5)
    axes[2, 2].set_ylabel('dwz (rad/s²)')
    axes[2, 2].set_title('Angular Accel Z')
    axes[2, 2].grid(True, alpha=0.3)
    
    # Row 4: Linear acceleration (dvx, dvy, dvz) - includes gravity compensation
    axes[3, 0].plot(t, data['dvx'], 'r-', linewidth=0.5)
    axes[3, 0].set_ylabel('dvx (m/s²)')
    axes[3, 0].set_title('Linear Accel X (+ g)')
    axes[3, 0].set_xlabel('Time (s)')
    axes[3, 0].grid(True, alpha=0.3)
    
    axes[3, 1].plot(t, data['dvy'], 'g-', linewidth=0.5)
    axes[3, 1].set_ylabel('dvy (m/s²)')
    axes[3, 1].set_title('Linear Accel Y (+ g)')
    axes[3, 1].set_xlabel('Time (s)')
    axes[3, 1].grid(True, alpha=0.3)
    
    axes[3, 2].plot(t, data['dvz'], 'b-', linewidth=0.5)
    axes[3, 2].set_ylabel('dvz (m/s²)')
    axes[3, 2].set_title('Linear Accel Z (+ g)')
    axes[3, 2].set_xlabel('Time (s)')
    axes[3, 2].grid(True, alpha=0.3)
    
    # Column 4: Torque and summary plots
    axes[0, 3].plot(t, data['tau6'], 'k-', linewidth=0.5)
    axes[0, 3].set_ylabel('tau6 (Nm)')
    axes[0, 3].set_title('Joint 6 Commanded Torque')
    axes[0, 3].grid(True, alpha=0.3)
    
    # Angular velocity magnitude
    w_mag = np.sqrt(data['wx']**2 + data['wy']**2 + data['wz']**2)
    axes[1, 3].plot(t, w_mag, 'm-', linewidth=0.5)
    axes[1, 3].set_ylabel('|ω| (rad/s)')
    axes[1, 3].set_title('Angular Velocity Magnitude')
    axes[1, 3].grid(True, alpha=0.3)
    
    # Linear velocity magnitude
    v_mag = np.sqrt(data['vx']**2 + data['vy']**2 + data['vz']**2)
    axes[2, 3].plot(t, v_mag, 'c-', linewidth=0.5)
    axes[2, 3].set_ylabel('|v| (m/s)')
    axes[2, 3].set_title('Linear Velocity Magnitude')
    axes[2, 3].grid(True, alpha=0.3)
    
    # Linear acceleration magnitude
    a_mag = np.sqrt(data['dvx']**2 + data['dvy']**2 + data['dvz']**2)
    axes[3, 3].plot(t, a_mag, 'orange', linewidth=0.5)
    axes[3, 3].set_ylabel('|a| (m/s²)')
    axes[3, 3].set_title('Linear Accel Magnitude')
    axes[3, 3].set_xlabel('Time (s)')
    axes[3, 3].grid(True, alpha=0.3)
    
    plt.suptitle('System ID Data - Link 6 (in link6 local frame)', fontsize=14)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"Saved plot to {output_path}")


def print_statistics(data):
    """Print basic statistics of the data."""
    print("\n=== Data Statistics ===")
    print(f"Duration: {data['time'][-1]:.2f} s")
    print(f"Samples: {len(data['time'])}")
    print(f"Sample rate: {1.0 / (data['time'][1] - data['time'][0]):.0f} Hz")
    print()
    
    signals = ['wx', 'wy', 'wz', 'vx', 'vy', 'vz', 
               'dwx', 'dwy', 'dwz', 'dvx', 'dvy', 'dvz', 'tau6']
    
    print(f"{'Signal':<8} {'Min':>12} {'Max':>12} {'Mean':>12} {'Std':>12}")
    print("-" * 60)
    for sig in signals:
        d = data[sig]
        print(f"{sig:<8} {d.min():>12.4f} {d.max():>12.4f} {d.mean():>12.4f} {d.std():>12.4f}")


if __name__ == "__main__":
    # Load data
    data_file = os.path.join(os.path.dirname(__file__), "system_id_link6_data.csv")
    
    if not os.path.exists(data_file):
        print(f"Error: Data file not found: {data_file}")
        print("Run system_id.py first to generate the data.")
        exit(1)
    
    data = load_data(data_file)
    
    # Print statistics
    print_statistics(data)
    
    # Generate plot
    plot_file = os.path.join(os.path.dirname(__file__), "system_id_link6_plot.png")
    plot_system_id_data(data, plot_file)
