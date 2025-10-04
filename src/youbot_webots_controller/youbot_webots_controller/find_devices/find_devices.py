#!/usr/bin/env python3
# find_devices.py
import sys
sys.path.append('/usr/local/webots/lib/controller/python')
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

print("=== ВСЕ УСТРОЙСТВА YOUBOT ===")
print(f"Количество устройств: {robot.getNumberOfDevices()}")

for i in range(robot.getNumberOfDevices()):
    device = robot.getDeviceByIndex(i)
    device_type = type(device).__name__
    print(f"{i:2d}. {device.getName()} - {device_type}")

print("\n=== ПОИСК УСТРОЙСТВ МАНИПУЛЯТОРА ===")
arm_devices = []
for i in range(robot.getNumberOfDevices()):
    device = robot.getDeviceByIndex(i)
    name = device.getName().lower()
    
    if any(keyword in name for keyword in ['arm', 'joint', 'finger', 'gripper', 'youbot']):
        device_type = type(device).__name__
        arm_devices.append((device.getName(), device_type))
        print(f"Найдено: {device.getName()} ({device_type})")

print("\n=== РЕКОМЕНДУЕМЫЕ ИМЕНА ===")
print("Моторы манипулятора:")
for name, dtype in arm_devices:
    if 'motor' in dtype.lower() or 'motor' in name.lower():
        print(f"  {name}")

print("\nДатчики положения:")
for name, dtype in arm_devices:
    if 'position' in dtype.lower() or 'sensor' in name.lower():
        print(f"  {name}")