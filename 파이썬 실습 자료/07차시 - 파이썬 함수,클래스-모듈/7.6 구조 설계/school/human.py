# human.py
class Human:
    def __init__(self, name, age, gender):
        self.name = name
        self.age = age
        self.gender = gender
    
    def introduce(self):
        print(f"안녕하세요, 제 이름은 {self.name}입니다. 나이는 {self.age}살이고, 성별은 {self.gender}입니다.")
    
    def have_birthday(self):
        self.age += 1
        print(f"생일 축하합니다! 이제 {self.age}살이 되었습니다.")
    
    def __str__(self):
        return f"Human(name={self.name}, age={self.age}, gender={self.gender})"
