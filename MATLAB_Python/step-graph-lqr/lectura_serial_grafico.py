import serial
import time
import csv
from datetime import datetime
import matplotlib.pyplot as plt
import pandas as pd

# Configura el puerto serial y la velocidad de transmisión (baud rate)
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.01)

def get_filename():
    """Genera un nombre de archivo basado en la fecha y hora actuales."""
    now = datetime.now()
    return f"data_{now.strftime('%Y%m%d_%H%M%S')}"

def save_data():
    filename = get_filename() + ".csv"
    with open(filename, mode="w", newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Time", "U", "Y", "X1", "X2"])  # Escribir encabezado
        print(f"Guardando datos en {filename}...")

        while True:
            line = ser.readline().strip()  # Leer línea del puerto serial
            if line:
                decoded_line = line.decode('utf-8')
                if decoded_line == "END":
                    break
                try:
                    # Separa los valores por comas
                    values = list(map(float, decoded_line.split(',')))
                    writer.writerow(values)  # Escribir datos en el CSV
                    print(decoded_line)  # Imprimir los datos recibidos
                except ValueError:
                    print(f"Error en el formato de la línea recibida: {decoded_line}")

    return filename

def plot_data(filename):
    """Lee los datos del archivo CSV y los guarda como imágenes de gráficas."""
    data = pd.read_csv(filename)  # Lee el archivo CSV con pandas

    # Utilizar la columna de tiempo del CSV para las gráficas
    time_data = data["Time"]
    u_data = data["U"]
    y_data = data["Y"]
    x1_data = data["X1"]
    x2_data = data["X2"]

    # Corregir primer valor
    u_data[0] = 0.0
    y_data[0] = 0.0
    x1_data[0] = 0.0
    x2_data[0] = 0.0

    # Graficar U
    plt.figure()
    plt.plot(time_data, u_data, label='u')
    plt.title('Señal de control (u) en el tiempo')
    plt.xlabel('Tiempo (s)')
    plt.ylabel('PWM (bit)')
    plt.grid()
    plt.legend()
    plt.savefig(filename.replace('.csv', '_U.png'))
    plt.close()

    # Graficar Y
    plt.figure()
    plt.plot(time_data, y_data, label='y', color='orange')
    plt.title('Salida (y) en el tiempo')
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Tensión de retroalimentación (V)')
    plt.grid()
    plt.legend()
    bottom, top = plt.ylim()
    plt.savefig(filename.replace('.csv', '_Y.png'))
    plt.close()

    # Graficar X1
    plt.figure()
    plt.plot(time_data, x1_data, label='x1', color='green')
    plt.title('Estado x1 en el tiempo')
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Valor de x1')
    plt.grid()
    plt.legend()
    #plt.ylim(bottom, top)
    plt.savefig(filename.replace('.csv', '_X1.png'))
    plt.close()

    # Graficar X2
    plt.figure()
    plt.plot(time_data, x2_data, label='x2', color='red')
    plt.title('Estado x2 en el tiempo')
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Valor de x2')
    plt.grid()
    plt.legend()
    plt.savefig(filename.replace('.csv', '_X2.png'))
    plt.close()

    print(f"Gráficas guardadas con éxito.")

if __name__ == "__main__":
    print("Esperando la palabra 'START' para empezar a guardar datos...")

    while True:
        line = ser.readline().strip()  # Leer línea del puerto serial como bytes
        if line:
            try:
                decoded_line = line.decode('utf-8')
                if decoded_line == "START":
                    print("Palabra 'START' recibida. Empezando a guardar datos...")
                    filename = save_data()
                    plot_data(filename)  # Llama a la función para guardar las gráficas
                    break
            except UnicodeDecodeError:
                print("Error al decodificar la línea.")
        time.sleep(0.01)  # Reduzca el tiempo de espera para mejorar la respuesta

    ser.close()  # Cerrar el puerto serial cuando el proceso termina
    print("Proceso completado. ¡Archivos guardados!")
