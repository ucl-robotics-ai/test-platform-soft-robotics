function setting3(app,a_3,f_3,d_3,st_3)
global time y3 n3 t


if app.pressure3ListBox.Value=="constant"
    [y,n]=constant_wave(time,a_3,f_3,d_3,st_3);
    y3=y;
    n3=n;
    
elseif app.pressure3ListBox.Value=="sine"
    [y,n]=sin_wave(time,a_3,f_3,d_3,st_3);
    y3=y;
    n3=n;
elseif app.pressure3ListBox.Value=="triangle"
    [y,n]=triangular_wave(time,a_3,f_3,d_3,st_3);
    y3=y;
    n3=n;
elseif app.pressure3ListBox.Value=="step"
    [y,n]=step_wave(time,a_3,f_3,d_3,st_3);
    y3=y;
    n3=n;
elseif app.pressure3ListBox.Value=="user define"
    n3=t;

           
end
i=1;
for i=1:size(n3)
if y3(i)<0.003
    y3(i)=0.003;
elseif y3(i)>2.997
    y3(i)=2.997;
else
    y3(i)=y3(i);
end
i=i+1;
end
end