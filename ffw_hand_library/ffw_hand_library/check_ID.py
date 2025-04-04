#!/usr/bin/env python3

from pymodbus.client import ModbusSerialClient
import time

PORT = '/dev/left_hand'
BAUDRATE = 115200

def scan_ids():
    print("🔍 Modbus ID 스캔 중...")

    client = ModbusSerialClient(port=PORT, baudrate=BAUDRATE, timeout=0.2)
    if not client.connect():
        print(f"❌ 시리얼 포트 {PORT} 연결 실패")
        return

    found_ids = []

    for slave_id in range(1, 10):
        try:
            result = client.read_holding_registers(address=0x0006, count=1, slave=slave_id)
            if not result.isError():
                found_ids.append(slave_id)
        except:
            pass
        time.sleep(0.01)

    client.close()

    if found_ids:
        print(f"\n🎯 사용 가능한 ID 리스트: {found_ids}")
    else:
        print("⚠️ 연결된 장치가 없습니다.")

if __name__ == '__main__':
    scan_ids()
