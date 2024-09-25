import time
import csv
from datetime import datetime
import matplotlib.pyplot as plt
import pandas as pd
import sys

filename = sys.argv[1]

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

plot_data(filename)
