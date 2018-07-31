from dill import load
src = open('obmap.pkl', 'rb')
obmap = load(src)
src.close()
