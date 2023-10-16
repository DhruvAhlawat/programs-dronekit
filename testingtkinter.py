import tkinter as tk;
def key(event):
    print(event.keysym); 
    if(event.keysym == 'Up'):
        print("PRESSING UP");     

root = tk.Tk(); 
print("doin it");
root.bind_all('<Key>', key)
root.mainloop(); 
