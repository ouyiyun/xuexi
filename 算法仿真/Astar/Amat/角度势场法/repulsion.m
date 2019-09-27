%   �������
%   ����ע�������ϰ�����att=0.01
%   ����ע���ڵ��ϰ����㹻��Զ��att=1/(2-0.2)=0.56
%   ����ע���ڵ��ϰ���̫����att=1000
%   Dsr=0.2%   
%   Dm=2

function max_rep=repulsion(caa, ob_transform)
m=size(ob_transform,1);
max_rep=0;
for i=1:m
    ob_k=ob_transform(i, 2)/ob_transform(i, 1);
    ob_angle=rad2deg(atan(ob_k));
    if ob_angle<0
        ob_angle=ob_angle+180;
    end
    if ob_angle>=caa(1) && ob_angle<=caa(2)
        ob_dis=normest(ob_transform(i,:));
%         yita=deg2rad(abs(caa(1)+6-ob_angle));
        if ob_dis<=0.2      %�������
            rep=1000;
%         elseif ob_dis*sin(yita)<=0.4
%             rep=1000;
        elseif ob_dis>=2    %�����Զ
            rep=0.56;
        else
            rep=1/(ob_dis-0.2);
        end
        if  rep>max_rep
            max_rep=rep;
        end
    end
end
if max_rep==0
    max_rep=0.01;
end
end