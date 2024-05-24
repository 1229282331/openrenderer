import pickle
f=open(r'C:\vscode_files\openrenderer\scripts\MANO_SMPLX_vertex_ids.pkl', "rb")#文件所在路径
inf=pickle.load(f, encoding='latin1')#读取pkl内容
print (inf)
f.close()


inf=str(inf['left_hand'])[1:-1]
ft = open('test.txt', 'w')  
ft.write(inf) 
