import sys, time, cv2

#Get Video Camera and set resolution
captObj = cv2.VideoCapture(0)
captObj.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 640 );
captObj.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 480 );

#Create two windows
cv2.namedWindow('Affichage RGB') 
cv2.namedWindow('Affichage profondeur')

# Get image, write it to a file and show it in a windows
flags, img = captObj.read()
time.sleep(1)
cv2.imwrite('test.jpg',img)
cv2.imshow("Affichage RGB", img)

# Wait
cv2.waitKey(0)
cv2.destroyAllWindows()
