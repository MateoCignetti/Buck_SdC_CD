import serial
import time
import csv
from datetime import datetime

# Configura el puerto serial y la velocidad de transmisión (baud rate)
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.01)  # Ajusta el timeout a un valor bajo para reducir el delay

def get_filename():
    """Genera un nombre de archivo basado en la fecha y hora actuales."""
    now = datetime.now()
    return f"data_{now.strftime('%Y%m%d_%H%M%S')}.csv"

def save_data():
    """Lee datos del puerto serial y los guarda en un archivo CSV después de recibir 'INICIO'."""
    filename = get_filename()
    with open(filename, mode="w", newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Sensor"])  # Escribir encabezado
        print(f"Guardando datos en {filename}...")

        while True:
            line = ser.readline().strip()  # Leer línea del puerto serial como bytes
            if line:
                try:
                    decoded_line = line.decode('utf-8')
                    #if decoded_line.isdigit() or (decoded_line[0] == '-' and decoded_line[1:].isdigit()):
                    writer.writerow([decoded_line])  # Escribir dato en el archivo CSV
                    print(decoded_line)
                except UnicodeDecodeError:
                    print("Error al decodificar la línea.")
            # Detener la lectura cuando se recibe el comando de fin de datos
            if decoded_line == "END":
                break
            time.sleep(0.01)  # Reduzca el tiempo de espera para mejorar la respuesta

if __name__ == "__main__":
    print("Esperando la palabra 'INICIO' para empezar a guardar datos...")

    while True:
        line = ser.readline().strip()  # Leer línea del puerto serial como bytes
        if line:
            try:
                decoded_line = line.decode('utf-8')
                if decoded_line == "START":
                    print("Palabra 'INICIO' recibida. Empezando a guardar datos...")
                    save_data()
                    break
            except UnicodeDecodeError:
                print("Error al decodificar la línea.")
        time.sleep(0.01)  # Reduzca el tiempo de espera para mejorar la respuesta

    ser.close()  # Cerrar el puerto serial cuando el proceso termina
    print("Proceso completado. ¡Archivo guardado!")
