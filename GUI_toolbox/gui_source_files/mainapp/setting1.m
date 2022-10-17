function setting1(app,a_1,f_1,d_1,st_1)
global time y1 n1 t

if app.pressure1ListBox.Value=="constant"
    [y,n]=constant_wave(time,a_1,f_1,d_1,st_1);
    y1=y;
    n1=n;
elseif app.pressure1ListBox.Value=="sine"
    [y,n]=sin_wave(time,a_1,f_1,d_1,st_1);
    y1=y;
    n1=n;
elseif app.pressure1ListBox.Value=="triangle"
    [y,n]=triangular_wave(time,a_1,f_1,d_1,st_1);
    y1=y;
    n1=n;
elseif app.pressure1ListBox.Value=="step"
    [y,n]=step_wave(time,a_1,f_1,d_1,st_1);
    y1=y;
    n1=n;
elseif app.pressure1ListBox.Value=="user define"

    n1=t;


    
end
i=1;
for i=1:size(n1)
if y1(i)<0.003
    y1(i)=0.003;
elseif y1(i)>2.997
    y1(i)=2.997;
else
    y1(i)=y1(i);
end
i=i+1;
end
end