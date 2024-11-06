import logging
from time import strftime, localtime

class Logger:
    def __init__(self, name: str) -> None:
        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging.DEBUG)

        uuid_str = strftime("%Y-%m-%d-%H-%M-%S", localtime()) 
        tmp_file_name ='%s.log' % uuid_str

        self.fh = logging.FileHandler('/home/czh/attack/log_file/%s' % tmp_file_name)
        self.fh.setLevel(logging.DEBUG)

        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        self.fh.setFormatter(formatter)

        self.logger.addHandler(self.fh)
    
    def info(self, message):
        self.logger.info(message)
    
    def debug(self, message):
        self.logger.debug(message)

    def warning(self, message):
        self.logger.warning(message)

    def error(self, message):
        self.logger.error(message)

    def critical(self, message):
        self.logger.critical(message)

if __name__ == '__main__':
    logger1 = Logger('1')
    logger2 = Logger('2')
    logger1.debug('hello1')
    logger2.debug('hello2')