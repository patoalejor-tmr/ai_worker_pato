from library import InspireHand
import time

CURRENT_ID = 1      # 현재 설정된 ID
NEW_ID = 2          # 변경하고자 하는 새 ID
PORT = '/dev/left_hand'  # 시리얼 포트 경로 (환경에 맞게 변경)

def change_hand_id(current_id, new_id, port):
    print(f"🔧 ID {current_id} ➜ {new_id} 변경 시도 중...")

    # 현재 ID로 객체 생성
    hand = InspireHand(port=port, hand_id=current_id)

    # 1. ID 변경 명령 전송 (RAM)
    data = [new_id] + [-1] * 5  # 첫 번째 값만 새 ID
    res = hand.set_6val(addr=0x03E8, values=data, label="set_ID_RAM")
    if not res:
        print("❌ ID 변경 실패 (RAM)")
        return

    print("✅ ID 변경됨 (RAM 적용). 현재 ID는 이제", new_id)

    # 2. 안정화 시간 대기
    print("⏳ 안정화 대기 중...")
    time.sleep(0.2)

    # 3. Flash 저장 명령 (새 ID로 다시 연결)
    hand = InspireHand(port=port, hand_id=new_id)
    res = hand.set_save_flash()
    if res:
        print("💾 Flash 저장 완료 — ID가 영구 변경되었습니다!")
    else:
        print("❌ Flash 저장 실패 — 재부팅 시 ID가 원래대로 돌아갈 수 있습니다.")

if __name__ == '__main__':
    change_hand_id(CURRENT_ID, NEW_ID, PORT)
