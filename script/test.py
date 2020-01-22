s = {}
s['3'] = 1
s['5'] = 2
s['0'] = 2
s['6'] = 2

a = s['0']
print(a)


print(s.has_key('3') and s.has_key('5') and s.has_key('6'))
