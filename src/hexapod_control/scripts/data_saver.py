import pickle


class DataSaver(object):
    def __init__(self, data_folder):
        self.f_c1_rf = open(data_folder+'/c1_lf.txt', 'w')

    def dump_data(self, data):
        self.f_c1_rf.write(' '.join(str(i) for i in data))



if __name__ == '__main__':
    data_folder = '/home/jichen/paper11_ws/src/hexapod_control/scripts/motor_data'
    file_name = data_folder + '/c1_lf.txt'
    print(file_name)
    ds = DataSaver(data_folder)

    data = [0.01, 0.5]
    ds.dump_data(data)