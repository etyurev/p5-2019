import threading
import os
from py4j.java_gateway import JavaGateway



def fun1():
    #os.system("java -cp D:\\TestJava\\out\\production\\TestJava Debug")
    os.system("java -cp C:\\Users\\etyur\\Documents\\ROBOTICS_AAU\\5_SEMESTER\\P_5\\ConnectScannerJAVA ConnectScannerJNI")


testThread = threading.Thread(target=fun1)
testThread.start()
print("Scanner launched !!!!!!")