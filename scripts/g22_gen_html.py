#! /usr/bin/env python3.3

from os import popen

texfile = open('doc/g22_project_report.tex')
html = open('g22_project_report.html', 'w')

html.write('<!DOCTYPE html>' + '\n\n' + '<html>' + '\n\n' + '<head>' + '\n\n')
html.write('<title>' + 'CS296 Project Report' + '</title>' + '\n\n')
html.write('<style>' + '\n\n')
html.write('h1{' + '\n' + 'font-family:Arial,Helvetica,sans-serif;' + '\n' + 'color:red;' + '\n' + 'text-align:center;' + '\n' + 'margin-top:5%;' \
	+ '\n' + 'margin-bottom:5%;' + '\n' + '}' + '\n\n')
html.write('h2{' + '\n' + 'font-family:Arial,Helvetica,sans-serif;' + '\n' + 'color:blue;' + '\n' + 'text-align:center;' + '\n' + 'margin-top:2%;' \
	+ '\n' + 'margin-bottom:2%;' + '\n' + '}' + '\n\n')
html.write('h3{' + '\n' + 'font-family:Arial,Helvetica,sans-serif;' + '\n' + 'color:blue;' + '\n' + 'margin-left:10%;' + '\n' + '}' + '\n\n')
html.write('p{' + '\n' + 'font-family:Arial,Helvetica,sans-serif;' + '\n' + '\n' + 'text-align:justify;' + '\n' + 'margin-top:1%;' \
	+ '\n' + 'margin-bottom:1%;' + '\n' + 'margin-left:10%;' + '\n' + 'margin-right:10%;' + '\n' + '}' + '\n\n')
html.write('img{' + '\n' + 'display:block;' + '\n' + 'margin-left:auto;' + 'margin-right:auto;' + '\n' + '}' + '\n\n')
html.write('table, th, td{' + '\n' + '\n' + 'font-family:Arial,Helvetica,sans-serif;' + '\n' + 'text-align:left;' + '\n' + 'color:brown;' + '\n' + \
	'border:1px solid black;' + '\n' + 'border-collapse:collapse;' + 'margin-left:auto;' + '\n' + 'margin-right:auto;' + '\n' + '}' + '\n\n')
html.write('</style>' + '\n\n')
html.write('</head>' + '\n\n' + '</html>' + '\n\n')

html.write('<body>' + '\n\n')

para_tag = False
table_tag = False

for line in texfile:
	line = line.strip().split()
	line1 = ''
	for word in line:
		if word[0:8] == '\\texttt{':
			line1 = line1 + ' ' + word[8:-1]
		else:
			line1 = line1 + ' ' + word
	line = line1.strip()
	if line == '}':
		continue
	if line[0:1] == '~':
		continue
	elif line == '\\hline':
		table_tag = True
		html.write('<table>' + '\n\n' + '<tr>' + '\n')
		continue
	elif line == '\\end{tabular}':
		table_tag = False
		html.write('\n' + '</tr>' + '\n\n' '</table>' + '\n\n')
		continue
	elif table_tag:
		line = line.split('&')
		for word in line[:]:
			word = word.strip()
			if len(word) < 6:
				html.write('<th>' + word + '</th>' + '\n')
			elif word[-6] == '\\':
				html.write('<th>' + word[0:-10] + '</th>' + '\n' + '</tr>' + '\n\n' + '<tr>' + '\n')
			else:
				html.write('<th>' + word + '</th>' + '\n')
	elif line != '':
		if line[0] == '\\':
			if para_tag:
				html.write('</p>' + '\n\n')
				para_tag = False
			index = line.find('[');
			if index == -1:
				index = line.find('{')
			tag = line[1:index]
			if tag == 'title':
				html.write('<h1>' + line[index + 1:-1] + '</h1>' + '\n\n')
			if tag == 'section':
				html.write('<h2>' + line[index + 1:-1] + '</h2>' + '\n\n')
			if tag == 'subsection':
				html.write('<h3>' + line[index + 1:-1] + '</h3>' + '\n\n')
			if tag == 'includegraphics':
				index = line.find('{')
				if line[index + 4:index + 14] == 'plots/plot':
					html.write('<img src="' + line[index + 1:index + 14] + '0' + line[index + 14:-5] + '" width="50%" height="30%">' + '\n\n')
				else:
					html.write('<img src="' + line[index + 1:-5] + '" width="50%" height="30%">' + '\n\n')
		else:
			if not para_tag:
				html.write('<p>' + '\n')
				para_tag = True
			if line[-2] == '\\':
				line = line[0:-2]
			html.write(line + ' <br>' + '\n')

html.write('</body>')

html.close()

popen('mv -f g22_project_report.html doc/')