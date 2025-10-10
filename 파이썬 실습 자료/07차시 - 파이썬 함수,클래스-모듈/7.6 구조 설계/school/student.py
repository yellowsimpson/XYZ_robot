# student.py
from school import human as h

class Student(h.Human):
    def __init__(self, name, age, gender, grade,number, major):
        super().__init__(name, age, gender)
        self.number = number        
        self.grade = grade
        self.major = major
    
    def study(self):
        print(f"{self.name}은(는) {self.major}를 공부하고 있습니다.")
    
    def introduce(self):
        super().introduce()
        print(f"저는 {self.grade}학년 {self.number}번이고, 전공은 {self.major}입니다.")
    
    def __str__(self):
        return f"Student(name={self.name}, age={self.age}, gender={self.gender}, grade={self.grade}, major={self.major})"

# 예제 사용
if __name__ == "__main__":
    student = Student("홍길동", 17, "남성", 2,1, "컴퓨터공학")
    student.introduce()
    student.study()
    print(student)
