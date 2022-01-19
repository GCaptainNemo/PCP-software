#!/usr/bin/env python3



import cv2
ID = 0
while(1):
    # get a frame
    try:
        cap = cv2.VideoCapture(ID)

        ret, frame = cap.read()
    except Exception as e:
        print(ID, " error ")
    # show a frame
    if ret == False:
        ID += 1
        continue
    elif ret:
        cv2.imshow("capture", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
cap.release()
cv2.destroyAllWindows() 


# import cv2

# #print("Before URL")
# cap = cv2.VideoCapture('rtsp://admin:a12345678@192.168.1.64/1')
# #print("After URL")

# while True:

#     #print('About to start the Read command')
#     ret, frame = cap.read()
#     #print('About to show frame of Video.')
#     cv2.imshow("Capturing",frame)
#     #print('Running..')

#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

cap.release()
cv2.destroyAllWindows()