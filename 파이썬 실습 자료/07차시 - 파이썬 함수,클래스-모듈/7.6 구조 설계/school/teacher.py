# student.py
from school import human as h

class Teacher(h.Human):
    def __init__(self, name, age, gender, grade, major):
        super().__init__(name, age, gender)
        self.grade = grade
        self.major = major
    
    def study(self):
        print(f"{self.name}은(는) {self.major}를 가르치고 있습니다.")
    
    def introduce(self):
        super().introduce()
        print(f"저는 {self.grade}학년이고, 전공은 {self.major}입니다.")
    
    def __str__(self):
        return f"Student(name={self.name}, age={self.age}, gender={self.gender}, grade={self.grade}, major={self.major})"

# 예제 사용
if __name__ == "__main__":
    teacher = Teacher("이영자", 27, "여성", 2, "컴퓨터공학")
    teacher.introduce()
    teacher.study()
    print(teacher)