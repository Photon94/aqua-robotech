from _typeshed import Self
import re
import pymurapy as mur

class robot():

    # Моторы 1 и 2 - вперед (отрицательное значение) и назад (положительное значение)
    # Моторы 0 и 3 - вниз (отрицательное значение) и вверх (положительное значение)

    def __init__(self, power:int) -> None:
        self.__body = mur.mur_init()
        self.__default_power:int = power


    def _turn_left(self, t:int, pow:int = None) -> None:
        if pow == None:
            pow = self.default_power

        self.__body.set_motor_power(1, -pow)
        self.__body.set_motor_power(2, pow)


    def _turn_right(self, t:int, pow:int = None) -> None:
        if pow == None:
            pow = self.default_power

        self.__body.set_motor_power(1, pow)
        self.__body.set_motor_power(2, -pow)


    def _forward(self, t:int, pow:int = None) -> None:
        if pow == None:
            pow = self.default_power
        
        self.__body.set_motor_power(1, -pow)
        self.__body.set_motor_power(2, -pow)


    def _backward(self, t:int, pow:int = None) -> None:
        if pow == None:
            pow = self.default_power

        self.__body.set_motor_power(1, pow)
        self.__body.set_motor_power(2, pow)


    def _down(self, t:int, pow:int = None) -> None:
        if pow == None:
            pow = self.default_power
        
        self.__body.set_motor_power(0, -pow)
        self.__body.set_motor_power(3, -pow)


    def _up(self, t:int, pow:int = None) -> None:
        if pow == None:
            pow = self.default_power
    
        self.__body.set_motor_power(0, pow)
        self.__body.set_motor_power(3, pow)


    def _run_func_by_name_str(self, name:str, arg1:str, arg2:str = None):
        if name == 'вперед':
            if arg2 == None:
                self._forward(int(arg1))
            else:
                self._forward(int(arg1), int(arg2))
        elif name == 'назад':
            if arg2 == None:
                self._backward(int(arg1))
            else:
                self._backward(int(arg1), int(arg2))
        elif name == 'влево':
            if arg2 == None:
                self._turn_left(int(arg1))
            else:
                self._turn_left(int(arg1), int(arg2))
        elif name == 'вправо':
            if arg2 == None:
                self._turn_right(int(arg1))
            else:
                self._turn_right(int(arg1), int(arg2))
        elif name == 'вниз':
            if arg2 == None:
                self._down(int(arg1))
            else:
                self._down(int(arg1), int(arg2))
        elif name == 'вверх':
            if arg2 == None:
                self._up(int(arg1))
            else:
                self._up(int(arg1), int(arg2))



    def run_sequence(self, order_path:str='order.ord'):
        # Очередь действий задается данными, которые содержатся в файле по пути order_path
        # Очередь действий записывается в многострочном виде
        # Каждое действие записывается отдельной строкой
        # Каждая строка имеет вид:
        # Д:<действие>, В:<время>[, М:<мощность>] 
        # действия: "вверх", "вниз", "вправо", "влево", "вперед", "назад"
        # время - время действия в секундах
        # мощность - мощность движетелей для действия в единицах от -100 до 100

        try:
            with open(order_path, 'r') as order_file:
                order_text:str = order_file.read
                order_list_str:list(str) = order_text.split('\n')
                for i in range(len(order_list_str)):
                    order_list_str[i].replace(' ', '')
                    order_list_str[i].replace('ё', 'е')
                    order_list_str[i].lower()
                re_pattern = r'д:([а-я]+)+,в:(\d+)+(,м:\d+)?'
                for order in order_list_str:
                    matches = re.match(re_pattern, order)
                    order_name = matches.group(1)
                    time_val = matches.group(2)
                    if (len(matches.group(3))!=None):
                        pow_val = matches.group(3)
                        self._run_func_by_name_str(order_name, time_val, pow_val)
                    else:
                        self._run_func_by_name_str(order_name, time_val)
        except:
            print('Ошибка чтения файла с инструкциями')
            exit(-1)

        pass