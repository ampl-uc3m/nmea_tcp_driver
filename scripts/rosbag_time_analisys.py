#! /usr/bin/env python3
import sys
import argparse
import os
import time
import rosbag
import pandas as pd
import matplotlib.pyplot as plt

def arg_read():
    parser = argparse.ArgumentParser(description="Analizador de rosbags con lidar, camara o ambas")
    parser.add_argument("bagname", type=str, help="Path en el que se encuentra el bag, sea carpeta de splits o ruta de bagfile")
    parser.add_argument("--lidartopics", type=str, nargs='+', help="Topic de los mensajes poincloud de los lidar")
    parser.add_argument("--cameratopics", type=str, nargs='+', help="Topic de los mensajes img de las cámaras")
    parser.add_argument("--measure", type=str, nargs='+', help="Representa la gráfica de tiempo y frecuencia para los topics")
    parser.add_argument("--compare", type=str, nargs='+', help="Representa la gráfica comparando los tiempos y frecuencia de los topics, junto una gráfica del error respecto al primer topic")
    args = parser.parse_args()

    if len(sys.argv) >= 2:
        bag_name = args.bagname
        lidar_topics = args.lidartopics if args.lidartopics else []
        camera_topics = args.cameratopics if args.cameratopics else []
        measures = args.measure if args.measure else []
        compare = []
        if args.compare:
            for par in args.compare:
                compare.append(par.split("&"))


    print("Nombre del bag:", bag_name)
    print("Topics de lidar:", lidar_topics)
    print("Topics de cámara:", camera_topics)


    return bag_name, lidar_topics, camera_topics, measures, compare

def readbag(bagfile,topics,times):

    baglen = 0

    with rosbag.Bag(bagfile) as bag:
        baglen = ((bag.get_end_time()-bag.get_start_time())/60)
        for topic, msg, t in bag.read_messages(topics=topics):
            #time.sleep(0.1)
            times[topic].append(int(msg.header.stamp.secs)*1000000000+int(msg.header.stamp.nsecs))
            
    for topic in topics:
        if len(times[topic]<1):
            print("No "+ topic + " msgs")
            del times[topic]
    

    return baglen

def mergebags(dir_path, topics, times):

    contenido = os.listdir(dir_path)
    contenido.sort()

    times = {topic: [] for topic in topics}
    baglen = 0

    for fichero in contenido:
        file_path = os.path.join(dir_path, fichero)
        if os.path.isfile(file_path) and fichero.endswith('.bag'):
            print("Reading " + fichero)
            new_baglen = readbag(file_path,topics ,times)
            baglen += new_baglen
            print('New bagfile merged\n')
    
    return baglen

def create_df(times):
    samples = []
    for topic in times.keys():
        samples.append(len(times[topic]))
    min_samples = min(samples)
    print('Number of samples: ' + str(min_samples))

    for topic in times.keys():
        times[topic]= times[topic][:min_samples]
    
    df = pd.DataFrame(data)
    #print(df.info())

    return df

def shift_calc(times, topics):
    #Calculo del desplazamiento de las capturas de tiempos
    #Se calcula respecto al primer topic, topic de lidar

    shifts = {topic: [] for topic in topics}

    for topic in topics:
        if len(times[topic])>20:
            shift_array = []
            for i in range(0,21):
                shift_array.append(abs(int(times[topics[0]][10])-int(times[topic][i])))
            
            shifts[topic] = 10 - shift_array.index(min(shift_array))

        else:
            print(f'Not enought {topic} samples for sift calc')
            exit()


    return shifts

def sift_correct(df,shifts):

    for topic, shift in shifts:
        if shift:
            df[topic] = df[topic].shift(shift)



if __name__ == "__main__":
    #t_objetivo =1000/10
    times = {}
    topics = {}

    bag_name, lidar_topics, camera_topics, measures, compare = arg_read()
    
    topics["lidar_topics"]=lidar_topics
    topics["camera_topics"]=camera_topics

    lidar_found = False
    cams_found = False
    csv_found = False
    
    if not os.path.isfile(bag_name):
        contenido = os.listdir(bag_name)
        for fichero in contenido:
            file_path = os.path.join(bag_name, fichero)
            if os.path.isfile(file_path) and fichero.endswith('merged.csv'):
                print('Reading csv')
                csv_found = True
                df = pd.read_csv(file_path)
                break
        if not csv_found:
            print ('No csv found')
            baglen = mergebags(bag_name, topics.values(),times)
            filename = bag_name + 'merged.csv'
    elif bag_name.endswith('.bag'):
        try:
            df = pd.read_csv(bag_name[:-4] + '.csv')
            print('Reading csv bagfile')
            csv_found = True
        except FileNotFoundError:
            print('No csv found')
            csv_found = False
            baglen = readbag(bag_name, topics.values() ,times)
            filename = bag_name[:-4] + '.csv'

    if not csv_found:
        print('\nBag duration: '+str(baglen) + ' min\n')

        df = create_df(times)

        shifts = shift_calc(times, topics)
        sift_correct(df, shifts)

        #Pasando los ts a ms
        for col in df:
            df[col] = df[col]/1000000

        df.to_csv(filename)
        print ('Saving data to: ' + filename)

    else:
        baglen = (df.iloc[-1, 0] -df.iloc[0, 0])/(60*1000)
        print('\nBag duration: '+str(baglen) + ' min\n')

        

    ### Métricas de tiempos de cámaras ###
    if cams_found:
        df['time'] = df['front_cam'] - df['front_cam'].iloc[0]
        # Comparación entre cámaras
        df['front_right']= df['front_cam']-df['right_cam']
        df['front_left']= df['front_cam']-df['left_cam']
        df['left_right']= df['left_cam']-df['right_cam']
        # Comparación entre cámara y ptp
        # df['cam_ros_front']= df['front_cam']-df['front_ros']
        # df['cam_ros_left'] = df['left_cam']-df['left_ros']
        # df['cam_ros_right']= df['right_cam']-df['right_ros']
        # Tiempo de captura de camaras
        df['front_cap_time'] = (df['front_cam']-df['front_cam'].shift(1))
        df['right_cap_time'] = (df['right_cam']-df['right_cam'].shift(1))
        df['left_cap_time'] = (df['left_cam']-df['left_cam'].shift(1))
        # Frecuencia de cámaras
        df['front_freq'] = 1000/df['front_cap_time']
        df['right_freq'] = 1000/df['right_cap_time']
        df['left_freq'] = 1000/df['left_cap_time']
        # Error cámaras
        df['front_error'] = df['front_cap_time'] - t_objetivo
        df['right_error'] = df['right_cap_time'] - t_objetivo
        df['left_error'] = df['left_cap_time'] - t_objetivo
        # Errores acumulados de cámaras
        df['front_cumsum'] = df['front_error'].cumsum()
        df['right_cumsum'] = df['right_error'].cumsum()
        df['left_cumsum'] = df['left_error'].cumsum()
        # Error acumulado total
        front_error_sum = df['front_error'].sum()
        right_error_sum = df['right_error'].sum()
        left_error_sum = df['left_error'].sum()
        # Desviaciones típicas de cámaras
        front_dev = df['front_cap_time'].std(ddof=0)
        right_dev = df['right_cap_time'].std(ddof=0)
        left_dev = df['left_cap_time'].std(ddof=0)
        # Tiempo medio de captura
        front_media = df['front_cap_time'].mean()
        right_media = df['right_cap_time'].mean()
        left_media = df['left_cap_time'].mean()
        # Frecuencias medias
        avg_front_freq = df['front_freq'].mean()
        avg_right_freq = df['right_freq'].mean()
        avg_left_freq = df['left_freq'].mean()

        print('\n\n\tFront camera\n')
        print('Media:\t' + str(front_media) +'ms\nFrecuencia media:\t' + str(avg_front_freq)+ 'Hz\nError acumulado:\t' + str(front_error_sum) + 'ms\nDesviación típica:\t'+str(front_dev))

        print('\n\n\tRight camera\n')
        print('Media:\t' + str(right_media) +'ms\nFrecuencia media:\t' + str(avg_right_freq)+ 'Hz\nError acumulado:\t' + str(right_error_sum) + 'ms\nDesviación típica:\t'+str(right_dev))

        print('\n\n\tLeft camera\n')
        print('Media:\t' + str(right_media) +'ms\nFrecuencia media:\t' + str(avg_right_freq)+ 'Hz\nError acumulado:\t' + str(right_error_sum) + 'ms\nDesviación típica:\t'+str(right_dev))

        fig1 = plt.figure("Desviaciones",figsize=(20,15))
        fig1.subplots_adjust(hspace=0.3)

        ax = fig1.add_subplot(2, 1, 1)
        ax.set_title('Deviation between cameras')
        ax.set_ylabel('ms')
        ax.plot(df['front_right'],label = 'Front-Right')
        ax.plot(df['front_left'], label = 'Front-Left')
        ax.plot(df['left_right'], label = 'Left-Right')
        ax.legend()

        ax = fig1.add_subplot(2, 1, 2)
        ax.set_title('Deviation PTP-ROS')
        ax.set_ylabel('ms')
        # ax.plot(df['cam_ros_front'], label = 'Front')
        # ax.plot(df['cam_ros_left'], label = 'Left')
        # ax.plot(df['cam_ros_right'], label = 'Right')
        ax.legend()

    ### Métricas de tiempos de Velodyne ###
    if lidar_found:
        df['time'] = df['velodyne'] - df['velodyne'].iloc[0]

        df['velo_cap_time'] = (df['velodyne']-df['velodyne'].shift(1))
        df['velo_freq'] = 1000/df['velo_cap_time']
        df = df.drop(0)
        #print(df)

        media = df['velo_cap_time'].mean()
        df['error'] = df['velo_cap_time'] - t_objetivo
        df['error_acumulado'] = df['error'].cumsum()
        error_sum = df['error'].sum()
        desv = df['velo_cap_time'].std(ddof=0)
        avg_velo_freq = df['velo_freq'].mean()

        print('\n\n\tVelodyne\n')
        print('Media:\t' + str(media) +'ms\nFrecuencia media:\t' + str(avg_velo_freq)+ 'Hz\nError acumulado:\t' + str(error_sum) + 'ms\nDesviación típica:\t'+str(desv))

        fig2 = plt.figure("Tiempo velodine",figsize=(20,20))
        fig2.subplots_adjust(hspace=0.3)

        ax = fig2.add_subplot(3, 1, 1)
        ax.set_title('Tiempo entre capturas')
        ax.set_ylabel('ms')
        ax.plot(df['velo_cap_time'],label = 'Tiempo')
        ax.legend()

        ax = fig2.add_subplot(3, 1, 2)
        ax.set_title('Frecuencia')
        ax.set_ylabel('Hz')
        ax.plot(df['velo_freq'], label = 'Freq')
        ax.legend()

        ax = fig2.add_subplot(3, 1, 3)
        ax.set_title('Error acumulado')
        ax.set_ylabel('Error (ms)')
        ax.plot(df['error_acumulado'], label = 'driff')
        ax.legend()

    if lidar_found and cams_found:
        df['velo_front'] = df['front_cam'] - df['velodyne']
        df['velo_left'] = df['left_cam'] - df['velodyne']
        df['velo_right'] = df['right_cam'] - df['velodyne']

        plt.figure("'Deviation camera - lidar'",figsize=(20,15))
    
        plt.ylabel('ms')
        plt.plot(df['velo_front'],label = 'Lidar-Front')
        plt.plot(df['velo_left'], label = 'Lidar-Left')
        plt.plot(df['velo_right'], label = 'Lidar-Right')
        plt.legend()

    plt.show()

    df.to_csv(bag_name+"metrics.csv")