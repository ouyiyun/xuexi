function K=Func_Alpha_Pos(Xb,Yb,Xn,Yn)
AngleY=Yn-Yb;
AngleX=Xn-Xb;
%***��Angle*******%
if Xb==Xn
    if Yn>Yb
        K=pi/2;
    else
        K=3*pi/2;
    end
else
    if Yb==Yn
        if Xn>Xb
            K=0;
        else
            K=pi;
        end
    else
        K=atan(AngleY/AngleX);
    end    
end
%****����K,ʹ֮��0~360��֮��*****%
   if (AngleY>0&&AngleX>0)%��һ����
        K=K;
    elseif (AngleY>0&&AngleX<0)||(AngleY<0&&AngleX<0)%�ڶ���������
        K=K+pi;
    else if (AngleY<0&&AngleX>0)%��������
            K=K+2*pi;  
        else
            K=K;
        end
    end
end