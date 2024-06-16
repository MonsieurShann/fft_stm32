import serial
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

class PcRemote:
    def __init__(self, port, limit):
        self.meas = []
        self.bench = []
        self.limit = limit
        self.port = port
        self.com = None
        self.listen()

    def init_com(self):
        return serial.Serial(self.port , 115200, timeout=1000)
    
    def listen(self):
        data = None
        self.com = self.init_com()
        while not data:
            try:
                data = self.com.readline().decode('latin-1').rstrip()  # Utilisation de 'latin-1' pour éviter les erreurs de décodage
                if (int)(data) == 1234:
                    self.benchmark()
                if (int)(data) == 5678:
                    self.fft()
            except UnicodeDecodeError as e:
                print(f"Unicode decode error: {e}")
    
    def benchmark(self):
        counter = 0
        while counter < 24:
            try:
                data = self.com.readline().decode('latin-1').rstrip()  # Utilisation de 'latin-1' pour éviter les erreurs de décodage
                self.bench.append(int(data))
                counter += 1
            except UnicodeDecodeError as e:
                print(f"Unicode decode error: {e}")
        self.plot_bench()

    
    def fft(self):
        counter = 0
        while counter < self.limit:
            try:
                data = self.com.readline().decode('latin-1').rstrip()  # Utilisation de 'latin-1' pour éviter les erreurs de décodage
                self.meas.append(int(data))
                counter += 1
            except UnicodeDecodeError as e:
                print(f"Unicode decode error: {e}")
        self.plot_fft()

    def plot_fft(self):
        plt.style.use('dark_background')
        x = [i for i in range(self.limit)]
        y = [self.meas[i]/1000000 for i in range(self.limit)]
        plt.plot(x, y, 'r')
        plt.show()

    def plot_bench(self):
        nb_fft = 100
        plt.style.use('dark_background')
        x = [pow(2, 4+i) for i in range(8)]
        y_1 = [self.bench[i]/nb_fft for i in range(8)]
        y_2 = [self.bench[i]/nb_fft for i in range(8, 16)]
        y_3 = [self.bench[i]/nb_fft for i in range(16, 24)]
        plt.plot(x, y_1, 'b', label='float 32')
        plt.plot(x, y_2, 'r', label='q15')
        plt.plot(x, y_3, 'g', label= 'q31')
        plt.legend()
        plt.title('Average time by fft in function of number of samples')
        plt.xlabel('Number of samples by fft')
        plt.ylabel('Time by fft (us)')
        plt.show()


pcr = PcRemote("/dev/ttyACM0", 150)

