import cv2

def main():
    device = "/dev/video2"

    cam = cv2.VideoCapture(device)
    img_num = 0

    while True:
        _, img = cam.read()

        key = cv2.waitKey(1)
        if key == ord('q'):
            break

        elif key == ord('s'):
            img_num += 1
            cv2.imwrite(f"img{img_num}.png", img)
            print("saved!")
        
        cv2.imshow("img", img)

    cam.release()

if __name__ == "__main__":
    main()