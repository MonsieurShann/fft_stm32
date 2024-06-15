import serial
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

class PcRemote:
    def __init__(self, port, limit):
        self.meas = []
        self.limit = limit
        self.counter = 0
        self.port = port
        self.com = None
        self.listen_port()

    def init_com(self):
        return serial.Serial(self.port , 115200, timeout=1000)
    
    def listen_port(self):
        self.com = self.init_com()
        while self.counter < self.limit:
            try:
                data = self.com.readline().decode('latin-1').rstrip()  # Utilisation de 'latin-1' pour éviter les erreurs de décodage
                self.meas.append(int(data))
                self.counter += 1
            except UnicodeDecodeError as e:
                print(f"Unicode decode error: {e}")
        print("Finish")

    def plot_results(self):
        plt.style.use('dark_background')
        x = [i for i in range(self.limit)]
        y = [self.meas[i]/1000000 for i in range(self.limit)]
        plt.plot(x, y, 'r')
        plt.show()


pcr = PcRemote("/dev/ttyACM0", 150)
pcr.plot_results()