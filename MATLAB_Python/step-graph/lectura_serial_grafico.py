import serial
import time
import csv
from datetime import datetime
import matplotlib.pyplot as plt  # Importa matplotlib para graficar
import pandas as pd  # Importa pandas para manejar datos en formato CSV
import numpy as np  # Importa numpy para manejar arreglos numéricos

# Configura el puerto serial y la velocidad de transmisión (baud rate)
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.01)  # Ajusta el timeout a un valor bajo para reducir el delay

def get_filename():
    """Genera un nombre de archivo basado en la fecha y hora actuales."""
    now = datetime.now()
    return f"data_{now.strftime('%Y%m%d_%H%M%S')}"

def save_data():
    """Lee datos del puerto serial y los guarda en un archivo CSV después de recibir 'START'."""
    filename = get_filename() + ".csv"
    with open(filename, mode="w", newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Sensor"])  # Escribir encabezado
        print(f"Guardando datos en {filename}...")

        while True:
            line = ser.readline().strip()  # Leer línea del puerto serial como bytes
            if line:
                try:
                    decoded_line = line.decode('utf-8')
                    if decoded_line == "END":
                        break
                    writer.writerow([decoded_line])  # Escribir dato en el archivo CSV
                    print(decoded_line)
                except UnicodeDecodeError:
                    print("Error al decodificar la línea.")
            time.sleep(0.01)  # Reduzca el tiempo de espera para mejorar la respuesta

    return filename  # Retorna el nombre del archivo para usarlo luego en la gráfica

def plot_data(filename):
    """Lee los datos del archivo CSV y los guarda como imagen de gráfica."""
    data = pd.read_csv(filename)  # Lee el archivo CSV con pandas
    output = data["Sensor"].astype(float)  # Convierte la columna 'Sensor' a flotante

    # Calcula el tiempo en base al número de muestras y la frecuencia de muestreo
    t = np.arange(0, len(output) * 200e-6, 200e-6)

    plt.figure()
    plt.plot(t, output)
    plt.title('Datos del Sensor')
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Valor del Sensor')
    plt.grid()

    # Guarda la gráfica en un archivo de imagen
    image_filename = filename.replace('.csv', '.png')
    plt.savefig(image_filename)
    plt.close()  # Cierra la figura para liberar memoria
    print(f"Gráfica guardada como {image_filename}")

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
                    plot_data(filename)  # Llama a la función para guardar la gráfica
                    break
            except UnicodeDecodeError:
                print("Error al decodificar la línea.")
        time.sleep(0.01)  # Reduzca el tiempo de espera para mejorar la respuesta

    ser.close()  # Cerrar el puerto serial cuando el proceso termina
    print("Proceso completado. ¡Archivo guardado!")
