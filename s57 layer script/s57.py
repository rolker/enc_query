from bs4 import BeautifulSoup
import requests
import re

urlPrefix = "http://www.s-57.com/Object.asp?nameAcr="



file = open("tags.txt")
content = file.read()
tagList = re.findall(r'(?<=\>)[A-Z].*[A-Z]+(?=\<)',content)
pointTags = []
for tag in tagList:
    url = urlPrefix + tag
    res = requests.get(url)
    text = res.text
    m = re.findall(r'(?<=Geometric primitives:\s).*(?=\<\/b)',text)
    if len(m) == 1 and m[0] == 'P':
        print([tag,m])
        pointTags.append(tag)

print(pointTags)
print((len(pointTags)))

# print(tagList)
# for n in m:
# u  = re.findall(r'[^,\s]+',n)
# print(u[0])