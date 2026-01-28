%% readme  %%
%坐标系定义如下：由y到x的为逆时针方向
%顺时针方向依次为x轴 encoder0 y轴 encoder1
%encoder0 和  y轴方向夹角为α0 
%encoder1 和  y轴方向夹角为α1 
%      [x , y]  *  [sina0(a)   -sina1(c)    =  [ e0 , e1]/k  k:脉冲/mm
%                   cosa0(b)    cosa1(d)]
%  需要两个不在同一条直线上的点   （做两次标定） 如果已知脉冲/mm  则只需要一个坐标
syms a b c d k;
index0 = input ('是否已知脉冲/mm Y or N: ','s');
if strcmpi(index0, 'n') || strcmpi(index0, 'N')
    index = input ('请输入坐标y0,x0 ,码盘脉冲e0,e1:','s');
    index = str2num(index);
    index1 = input ('请输入坐标y1,x1 ,码盘脉冲e0,e1:','s');
    index1 = str2num(index1);
    y0=index(1);
    x0=index(2);
    e00=index(3);
    e10=index(4);
    y1=index1(1);
    x1=index1(2);
    e01=index1(3);
    e11=index1(4);
    
    % 构建等式
    eq1 = (y0 * b + x0 * a) == e00/k;
    eq2 = (y0 * d + x0 * c) == e10/k;
    eq3 = (y1 * b + x1 * a) == e01/k;
    eq4 = (y1 * d + x1 * c) == e11/k;
    %eq5 = a^2 + b^2 == 1;
    %eq6 = c^2 + d^2 == 1;
    
    %constraints = [a > 0, b > 0, c < 0, d > 0];
    % 求解矩阵元素和标量
    %[sol_a, sol_b, sol_c, sol_d,sol_k] = solve([eq1, eq2, eq3, eq4, eq5, eq6 ,constraints], [a, b, c, d, k]);
    [sol_a, sol_b, sol_c, sol_d,sol_k] = solve([eq1, eq2, eq3, eq4], [a, b, c, d, k]);
    k=abs(sol_k);

else
    index = input ('请输入脉冲/mm: ','s');
    index = str2num(index);
    k=index;
    index = input ('请输入坐标y,x ,码盘脉冲e0,e1:','s');
    index = str2num(index);
    y=abs(index(1));
    x=abs(index(2));
    e0=index(3);
    e1=index(4);
    eq1 = (y * b + x * a) == abs(e0)/k;
    eq2 = (y * d + x * c) == -abs(e1)/k;
    eq3 = a^2 + b^2 == 1;
    eq4 = c^2 + d^2 == 1;
    constraints = [a > 0, b > 0, c < 0, d > 0];
    % 求解矩阵元素和标量
    [sol_a, sol_b, sol_c, sol_d] = solve([eq1, eq2, eq3, eq4,constraints], [a, b, c, d]);

end
xishu=[sol_a/k,sol_c/k;sol_b/k,sol_d/k];
a1=atand(sol_a/sol_b);
a2=atand(-1*sol_c/sol_d);
xishu=inv(xishu);
if strcmpi(index0, 'y') || strcmpi(index0, 'Y')
    xishu=abs(xishu);
end
xishu=double(xishu);
k=double(k);
% 输出结果
disp('系数矩阵:');
if strcmpi(index0, 'y') || strcmpi(index0, 'Y')
    disp('仅给出系数矩阵的绝对值，根据编码器读数再增加负号')
end
vpa(xishu,10) 
disp('脉冲/mm=');
vpa(k,10) 
s='a1=';   
disp(s)
disp(double(a1));
s='a2=';   
disp(s)
disp(double(a2));



