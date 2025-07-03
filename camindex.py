import cv2

def test_camera(index=1):
    print(f"🔍 Opening camera at index {index}...")
    cap = cv2.VideoCapture(index)

    if not cap.isOpened():
        print("❌ Failed to open camera.")
        return

    print("✅ Camera opened. Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️ Failed to grab frame. Exiting...")
            break

        cv2.imshow("📷 QPC1010 Camera Feed", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_camera(1)
