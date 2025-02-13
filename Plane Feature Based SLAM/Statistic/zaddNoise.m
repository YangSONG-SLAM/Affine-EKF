%coded by Yang SONG
function [z_noise,z0]=zaddNoise(z_expectation,z_expectation0,OBSV_noise)
T_steps=size(z_expectation,2);
z_noise=cell(1,T_steps);

N=size(z_expectation0.CPplane,2);
if N<1
%     z0.rotation=[];
    z0.CPplane=[];
else
    for j=1:N
        V=mvnrnd(zeros(1,3),OBSV_noise,1)';
%         normV=norm(V(1:3));
%         normvec=V(1:3)/normV;
%         if normV>=2*pi
%             normV=mod(normV,2*pi);
%             if normV>pi
%                 normV=normV-2*pi;
%             end
%         end
%         V(1:3)=normV*normvec;
%         z0.rotation(1:3,3*j-2:3*j)=so3_exp(V(1:3))*z_expectation0.rotation(1:3,3*j-2:3*j);
        z0.CPplane(1:3,j)=z_expectation0.CPplane(1:3,j)+V;
    end
end



for i=1:T_steps
    N=size(z_expectation{i}.CPplane,2);
    if N<1
%         z_noise{i}.rotation=[];
        z_noise{i}.CPplane=[];
    else
        for j=1:N
            V=mvnrnd(zeros(1,3),OBSV_noise,1)';
%             normV=norm(V(1:3));
%             normvec=V(1:3)/normV;
%             if normV>=2*pi
%                 normV=mod(normV,2*pi);
%                 if normV>pi
%                     normV=normV-2*pi;
%                 end
%             end
%             V(1:3)=normV*normvec;
%             z_noise{i}.rotation(1:3,3*j-2:3*j)=so3_exp(V(1:3))*z_expectation{i}.rotation(1:3,3*j-2:3*j);
            z_noise{i}.CPplane(1:3,j)=z_expectation{i}.CPplane(1:3,j)+V;
        end
    end
end


