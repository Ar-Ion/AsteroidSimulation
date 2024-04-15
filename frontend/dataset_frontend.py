from . import Frontend
import pickle
import os

class DatasetFrontend(Frontend):
    def __init__(self, folder, size):
        super().__init__()
        self._folder = folder
        self._size = size
        self._id = 0
        
    def on_start(self):
        print("Dataset frontend started")
            
    def on_stop(self):
        print("Dataset frontend stopped")

    def on_data(self, data):    
        if self._id < self._size:
            filename = os.path.join(self._folder, str(self._id).zfill(6) + ".pickle")
            self._id += 1

            with open(filename, 'wb') as handle:
                pickle.dump(data, handle, protocol=pickle.HIGHEST_PROTOCOL)
                
            if self._id % 100 == 0:
                print("Generated " + str(self._id) + " synthetic payloads")
            

        