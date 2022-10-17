function setting2(app,a_2,f_2,d_2,st_2)
global time y2 n2 t


if app.pressure2ListBox.Value=="constant"
    [y,n]=constant_wave(time,a_2,f_2,d_2,st_2);
    y2=y;
    n2=n;
elseif app.pressure2ListBox.Value=="sine"
    [y,n]=sin_wave(time,a_2,f_2,d_2,st_2);
    y2=y;
    n2=n;
elseif app.pressure2ListBox.Value=="triangle"
    [y,n]=triangular_wave(time,a_2,f_2,d_2,st_2);
    y2=y;
    n2=n;
elseif app.pressure2ListBox.Value=="step"
    [y,n]=step_wave(time,a_2,f_2,d_2,st_2);
    y2=y;
    n2=n;
elseif app.pressure2ListBox.Value=="user define"

    n2=t;
end

i=1;
for i=1:size(n2)
if y2(i)<0.003
    y2(i)=0.003;
elseif y2(i)>2.997
    y2(i)=2.997;
else
    y2(i)=y2(i);
end
i=i+1;
end
end