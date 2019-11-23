import numpy as np
import cv2

cam = cv2.VideoCapture(0)

def nothing(x):
    pass

def roi(image,vert):
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, vert, 255)
    masked = cv2.bitwise_and(image, mask)
    return masked

img = np.zeros([60, 200, 3], np.uint8)
cv2.namedWindow('image', cv2.WINDOW_NORMAL)
cv2.createTrackbar('H_up','image',101,255,nothing)
cv2.createTrackbar('S_up','image',239,255,nothing)
cv2.createTrackbar('V_up','image',210,255,nothing)
cv2.createTrackbar('H_lo','image',30,255,nothing)
cv2.createTrackbar('S_lo','image',39,255,nothing)
cv2.createTrackbar('V_lo','image',0,255,nothing)


while(1):
    _,frame = cam.read()
    #frame = cv2.imread("binary.jpg")
    """
    scale_percent = 20  # percent of original size
    width = int(frame.shape[1] * scale_percent / 100)
    height = int(frame.shape[0] * scale_percent / 100)
    dim = (width, height)
    # resize image
    frame = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
    """

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    Hu = cv2.getTrackbarPos('H_up', 'image') #103 - 95 - 101
    Su = cv2.getTrackbarPos('S_up', 'image') #239
    Vu = cv2.getTrackbarPos('V_up', 'image') #210
    Hl = cv2.getTrackbarPos('H_lo', 'image') #62 - 72 - 30
    Sl = cv2.getTrackbarPos('S_lo', 'image') #57 - 39
    Vl = cv2.getTrackbarPos('V_lo', 'image') #0

    lower_green = np.array([Hl, Sl, Vl])
    upper_green = np.array([Hu, Su, Vu])

    mask = cv2.inRange(hsv, lower_green, upper_green)
    res = cv2.bitwise_and(frame, frame, mask = mask)
    res = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

    #gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    #thresh = cv2.medianBlur(gray, 5)
    #thresh1 = cv2.adaptiveThreshold(res, 200, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 21, 12)
    _, contours, h  = cv2.findContours(res, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
        if len(approx) == 4 or len(approx) == 5 or len(approx) == 6:
            res1 = approx[len(approx)-1][0][0] * approx[0][0][1]
            res2 = approx[len(approx)-1][0][1] * approx[0][0][0]
            for i in range(len(approx)-1):
                res1 += approx[i][0][0]*approx[i+1][0][1]
                res2 += approx[i][0][1] * approx[i + 1][0][0]
            cal = 0.5*(abs(res1-res2))
            #print("rec")
            if(cal > 50):
                region = roi(frame, [cnt])
                cv2.imshow("ROI", cv2.cvtColor(region, cv2.COLOR_BGR2RGB))
                cv2.drawContours(frame, [cnt], 0, (25, 25, 255), 2)

        elif len(approx) > 12:
            #print("circle")
            cv2.drawContours(frame, [cnt], 0, (255, 25, 25), 3)
        #else:
            #print("not rec")
    cv2.imshow("Goruntu", res)
    cv2.imshow('image', img)
    cv2.imshow("Contours", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
cam.release()

# or len(approx) == 5 or len(approx) == 6