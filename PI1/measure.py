import numpy

class Measure :
    __data = []
    __min:float
    __max:float
    
    def __init__( self, min, max ) :
        self.clear()
        self.__min = min
        self.__max = max
    
    def clear( self ) :
        self.__data.clear()
    
    def add( self, value:float ):
        if self.__min <= value and value <= self.__max : 
            self.__data.append( value )
        
    def getAverage( self ) :
        avg = numpy.mean( self.__data )
        self.clear()
        return avg