from abc import ABC, abstractmethod
import multiprocessing as mp

class Frontend(ABC):
    @abstractmethod            
    def on_start(self):
        pass
    
    @abstractmethod            
    def on_stop(self):
        pass
    
    @abstractmethod            
    def on_data(self, data):
        pass

class AsyncEvents:
    START = 1
    STOP = 2

class AsyncFrontend:
    def __init__(self, frontend):
        mp.set_start_method('fork')
        self._data_queue = mp.Queue()
        self._event_queue = mp.Queue()
        self._process = mp.Process(target=AsyncFrontend.loop, args=(frontend, self._event_queue, self._data_queue))
        
    def start(self):
        self._process.start()
        self._event_queue.put(AsyncEvents.START)
        
    def stop(self):
        self._event_queue.put(AsyncEvents.STOP)
        self._process.join()
    
    def loop(frontend, event_queue, data_queue):
        while True:
            try:
                event = event_queue.get(False)
                
                if event == AsyncEvents.START:
                    frontend.on_start()
                elif event == AsyncEvents.STOP:
                    frontend.on_stop()
                    break
            except:
                pass # No new event
            
            data = data_queue.get()
            frontend.on_data(data)
            
    def feed_data(self, data):
        self._data_queue.put(data)