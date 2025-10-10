# student.py
from school import human as h
from school import student as s
from school import teacher as t

print('Hello This is Sejong School')


# 예제 사용
if __name__ == "__main__":
    student = s.Student("홍길동", 20, "남성", 2,1, "컴퓨터공학")
    student.introduce()
    student.study()
    
    print(student)
    
    
    teacher = t.Teacher("이영자", 27, "여성", 2, "컴퓨터공학")
    teacher.introduce()
    teacher.study()
    print(teacher)  