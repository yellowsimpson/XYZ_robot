class Person:    # 클래스  
    def __init__(self, name, age, address='충남'):  
        self.name = name  
        self.age = age  
        self.address = address  
    
    def greeting(self):  
        print('안녕하세요, 저는 {0}이고 {1}이고 {2}니다.'.format(self.name,self.age,self.address))