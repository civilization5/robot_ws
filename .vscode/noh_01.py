

# try:
#   number_input_a=int(input("정수입력:"))
#   print("원의반지름:", number_input_a)
#   print("둘레:", 2*3.14*number_input_a)
#   print("넓이:",3.14*number_input_a*number_input_a)
# except:
#   print("오류")

# list_input_a=["52","273","32","ㅇㅇ"]

# list_num=[]
# for i in list_input_a:
#   try:
#     float(i)
#     list_num.append(i)
#   except:
#     pass

# print("{} 내부에 있는 숫자는".format(list_input_a))
# print("{}".format(list_num))

# list_num=[52,23,27,10]
# try:
#   num_input=int(input("정수 입력:"))
#   print("{}번째 요소: {}".format(num_input,list_num[num_input]))
# except ValueError:
#   print("정수 입력하시오")
# except IndexError:
#   print("인덱스 벗어남")
# import math
# print(math.sin(1))

# from math import*
# print(sin(1))

# from urllib import request

# target= request.urlopen("https://www.google.com/?hl=ko&zx=1752024825286&no_sw_cr=1")
# output=target.read()
# print(output)

# from urllib import request
# from bs4 import BeautifulSoup

# target = request.urlopen("https://www.kma.go.kr/weather/forecast/mid-term-rss3.jsp?stnId=108")
# soup = BeautifulSoup(target, "html.parser")

# for location in soup.select("location"):
#     city = location.select_one("city").string
#     print("도시:", city)

#     # 여러 개의 날씨 정보가 있음
#     for data in location.select("data"):
#         date = data.select_one("tmEf").string
#         weather = data.select_one("wf").string
#         tmn = data.select_one("tmn").string
#         tmx = data.select_one("tmx").string

#         print("  날짜:", date)
#         print("  날씨:", weather)
#         print("  최저기온:", tmn)
#         print("  최고기온:", tmx)
#         print()
#     print("-" * 40)

# 클래스

# class Student():
#   def __init__(self, name, korean, math, english, science):
#     self.name = name
#     self.korean = korean
#     self.math = math
#     self.english = english
#     self.science = science

#   def get_sum(self):
#     return self.korean + self.math + self.english + self.science

#   def get_average(self):
#     return self.get_sum() / 4

#   def __str__(self):
#     return "{}\t{}\t{}".format(
#       self.name,
#       self.get_sum(),
#       self.get_average()
#     )
# students = [
#   Student("윤인성",92,90,80,70),
#   Student("김ㅇㅇ",95,85,75,54),
#   Student("박ㅇㅇ",32,42,43,52),
#   Student("최ㅇㅇ",43,53,23,12),
# ]
# print("이름","총점","평균", sep="\t")
# for student in students:
#   print(str(student))

# class Student():
#   def study(self):
#     print("공부하는중")

# class Teacher():
#   def teach(self):
#     print("학생을 가르칩니다.")























