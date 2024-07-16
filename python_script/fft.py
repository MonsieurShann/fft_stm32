import serial
import struct
import matplotlib.pyplot as plt

# Configuration du port série
ser = serial.Serial(
    port='/dev/ttyACM0',  # Remplacez par le port série approprié pour votre système
    baudrate=115200,  # Débit en bauds, doit correspondre à celui du microcontrôleur
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1  # Temps d'attente pour la lecture (en secondes)
)

def read_uart(sof, eof):
    recording = False
    STOP = False
    meas = []
    try:
        while not STOP:
            if ser.in_waiting > 0:
                data = ser.read(4)  # Lire 4 octets à la fois pour float32_t
                if len(data) == 4:
                    # Conversion des données reçues en float
                    received_value = struct.unpack('>I', data)[0]  # '>f' pour big-endian float
                    print(received_value)
                    if(recording == True):
                        meas.append(received_value/10000)

                    if(received_value == sof):
                        print('SOOOOOOOOOOOOOF')
                        recording = True

                    if(received_value == eof):
                        STOP = True
                        plot(meas)
    except KeyboardInterrupt:
        print("Exiting Program")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        ser.close()

def plot(meas):
    plt.style.use('dark_background')
    x = [0.5*i*2048/8192 for i in range(len(meas)-2)]
    y = meas[1: -1]
    plt.title('f sample = 2048 Hz, Num sample = 8192')
    plt.xlabel('Freq (Hz)')
    plt.ylabel('Amplitude')
    plt.axvline(x=50, color='red', linestyle='--', label='Fondamental')
    plt.plot(x, y)
    plt.show()

if __name__ == "__main__":
    read_uart(6969, 9999)
