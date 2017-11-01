clear;clc;
global serving_rate;
global cs_number;
global berth_number;
global buffer_number;
global jam_spacing;
global free_speed;
global back_speed;
global moveup_speed;
global cycle_length_number;
global green_ratio;
global sim_size;

berth_number = 2;
buffer_number = 3:1:3;

green_ratio = 0.5;
jam_spacing = 12;
free_speed = 20 / 3.6;
back_speed = 25 / 3.6;
moveup_speed = 20 / 3.6;
mh = jam_spacing / moveup_speed;
bh = jam_spacing / back_speed;
fh = jam_spacing / free_speed;
serving_rate = 1/(25);
% mh=0;
% bh=0;
% fh=0;

p=1; %time-space or not
% buffer_number = 2:2;
global cs;
cs = 0.45;
if cs==0
    sim_size = 1000;
else
    sim_size = 50000;
end

if p==1
    sim_size = 20;
    cycle_length_number = 120;
else
    cycle_length_number = 80:1:240;
end
% mh=0;
% bh=0;
% fh=0;

%% simulation procedures
cs_size = sum(cs_number~=-1);
c_size = sum(berth_number~=-1);
d_size = sum(buffer_number~=-1);
cl_size = sum(cycle_length_number~=-1);

unblocked=[];
capacity=[];

for l=1:cl_size
    cycle_length = cycle_length_number(l);
    green_time = cycle_length * green_ratio;
    red_time = cycle_length * (1-green_ratio);
    if cs==0
        serv_time(1 : sim_size) = unifrnd(1/serving_rate, 1/serving_rate);
        round(serv_time, 1);
    else
        serv_time(1 : sim_size) = gamrnd(1/cs^2, cs^2/serving_rate, 1, sim_size);
    end
    
    for k=1:c_size
        c = berth_number(k);
        for j=1:d_size
            u = zeros(sim_size, 1);
            berth_pos = zeros(sim_size, 1);
            buffer_pos = zeros(sim_size, 1);
            d = buffer_number(j);
            d0 = mod(d,c);
            
            deque_moment = zeros(sim_size, 1);
            end_serv_moment = zeros(sim_size, 1);
            
            wait_berth = zeros(sim_size, 1);
            leave_berth_moment = zeros(sim_size, 1);
            wait_buffer = zeros(sim_size, 1);
            leave_buffer_moment = zeros(sim_size, 1);
            
            %             if ((d+c)*fh + (d+c-1)*bh) >= green_time
            %                 disp('green_time is too short to allow all buses in red time to leave');
            %             end
            
            %wasted time stat
            for i=1:sim_size
                if i == 1
                    deque_moment(i) = bh;
                    berth_pos(i) = 1;
                    end_serv_moment(i) = deque_moment(i) + c*mh + serv_time(i);
                    wait_berth(i) = 0;
                    leave_berth_moment(i) = end_serv_moment(i);
                    if mod(deque_moment(i) + serv_time(i) + c*mh + d*fh,...
                            cycle_length) <= green_time  %??????????????G
                        buffer_pos(i) = 0;
                        u(i) = 0;
                        wait_buffer(i) = 0;
                        leave_buffer_moment(i) = deque_moment(i) + c*mh + serv_time(i);
                    else
                        buffer_pos(i) = 1;
                        u(i) = 1;
                        wait_buffer(i) = cycle_length - mod(deque_moment(i) + serv_time(i) + c*mh + d*fh, cycle_length) + bh;
                        leave_buffer_moment(i) = deque_moment(i) + c*mh + serv_time(i) + d*fh + wait_buffer(i);
                    end
                else %i>1
                    
                    if berth_pos(i-1) < c
                        deque_moment(i) = deque_moment(i-1) + (mh+bh)*1;
                        berth_pos(i) = berth_pos(i-1) + 1;
                    else
                        if berth_pos(i-1) == c
                            if u(i-1) < d+c
                                if u(i-1) < d
                                    berth_pos(i) = 1;
                                else
                                    berth_pos(i) = u(i-1) - d + 1;
                                end
                                
                                deque_moment(i) = deque_moment(i-1) + (c-berth_pos(i-1)+1)*mh + serv_time(i-1)...
                                    + wait_berth(i-1) + bh;
                                
                            else
                                berth_pos(i) = 1;
                                deque_moment(i) = leave_buffer_moment(i-1) + bh;
                            end
                        else
                            disp('wrong');
                        end
                    end
                    end_serv_moment(i) = deque_moment(i) + (c-berth_pos(i)+1)*mh + serv_time(i);
                    wait_berth(i) = max(0, leave_berth_moment(i-1) + bh-end_serv_moment(i));
                    leave_berth_moment(i) = end_serv_moment(i) + wait_berth(i);
                    
                    if u(i-1) == 0
                        if mod(leave_berth_moment(i) + (berth_pos(i)+d-1)*fh, cycle_length) <= green_time
                            buffer_pos(i) = 0;
                            u(i) = 0;
                            wait_buffer(i) = 0;
                            leave_buffer_moment(i) = leave_berth_moment(i);
                        else
                            buffer_pos(i) = 1;
                            u(i) = 1;
                            wait_buffer(i) = cycle_length + u(i)*bh - mod(leave_berth_moment(i) + (berth_pos(i)+d-1)*fh, cycle_length);
                            leave_buffer_moment(i) = leave_berth_moment(i) + (berth_pos(i)+d-u(i))*fh + wait_buffer(i);
                        end
                    else
                        if u(i-1) < d+c
                            if leave_buffer_moment(i-1)+bh <= leave_berth_moment(i)+(berth_pos(i)+(d-u(i-1))-1)*fh
                                if mod(leave_berth_moment(i) + (berth_pos(i)+d-1)*fh, cycle_length) <= green_time
                                    buffer_pos(i) = 0;
                                    u(i) = 0;
                                    wait_buffer(i) = 0;
                                    leave_buffer_moment(i) = leave_berth_moment(i);
                                else
                                    buffer_pos(i) = 1;
                                    u(i) = 1;
                                    wait_buffer(i) = cycle_length + u(i)*bh - mod(leave_berth_moment(i) + (berth_pos(i)+d-1)*fh, cycle_length);
                                    leave_buffer_moment(i) = leave_berth_moment(i) + (berth_pos(i)+d-u(i))*fh + wait_buffer(i);
                                end
                            else
                                buffer_pos(i) = buffer_pos(i-1) + 1;
                                u(i) = u(i-1) + 1;
                                wait_buffer(i) = leave_buffer_moment(i-1) + bh - leave_berth_moment(i) - (berth_pos(i)+d-u(i))*fh;
                                %                                 leave_buffer_moment(i) = leave_berth_moment(i) + (berth_pos(i)+d-u(i))*fh + wait_buffer(i);
                                leave_buffer_moment(i) = leave_buffer_moment(i-1) + bh;
                            end
                        else
                            if u(i-1) == d+c
                                if mod(leave_berth_moment(i) + (berth_pos(i)+d-1)*fh, cycle_length) <= green_time
                                    buffer_pos(i) = 0;
                                    u(i) = 0;
                                    wait_buffer(i) = 0;
                                    leave_buffer_moment(i) = leave_berth_moment(i);
                                else
                                    buffer_pos(i) = 1;
                                    u(i) = 1;
                                    wait_buffer(i) = cycle_length + buffer_pos(i)*bh - mod(leave_berth_moment(i) + (berth_pos(i)+d-u(i))*fh, cycle_length);
                                    leave_buffer_moment(i) = leave_berth_moment(i) + (berth_pos(i)+d-u(i))*fh + wait_buffer(i);
                                end
                            else
                                disp('impossible to be bigger than d+c');
                            end
                        end
                    end
                end
            end
            end_time = deque_moment(sim_size)+serv_time(sim_size)+wait_berth(sim_size)+wait_buffer(sim_size);
            capacity(l,j) = 3600 * sim_size / end_time;
            %unblocked(l,j) = 1 - total_waste/end_time;
        end
    end
end

if p==0
    plot(cycle_length_number,capacity,'linewidth',2,'linestyle','-','color',[111/255, 122/255, 117/255]);
    hold on;
end



if p
    plot_number = sim_size;
    
    for i=1:c+d
        %plot berth and buffer
        x = [0,leave_berth_moment(plot_number)];
        y = [jam_spacing*i, jam_spacing*i];
        plot(x,y,'--');
        hold on;
    end
    
    %plot signal
    for i=1:ceil(leave_berth_moment(plot_number)/cycle_length);
        signal_y = [(c+d+1)*jam_spacing,(c+d+1)*jam_spacing];
        signal_red_x = [(i-1)*cycle_length,(i-1)*cycle_length+green_time];
        signal_green_x = [(i-1)*cycle_length+green_time,i*cycle_length];
        line(signal_red_x,signal_y,'Color','g','LineWidth',4,'LineStyle','-');
        line(signal_green_x,signal_y,'Color','r','LineWidth',4,'LineStyle','-');
    end
    
    for i=1:plot_number
        %1
        deque_x = [deque_moment(i), deque_moment(i)+(berth_number-berth_pos(i)+1)*mh];
        deque_y = [0,(berth_number-berth_pos(i)+1)*jam_spacing];
        line(deque_x, deque_y, 'linewidth',2);
        
        %2
        serv_x = [deque_x(2), end_serv_moment(i)];
        serv_y = [deque_y(2),(berth_number-berth_pos(i)+1)*jam_spacing];
        line(serv_x,serv_y,'linewidth',5);
        
        %3
        wait_berth_x = [serv_x(2),end_serv_moment(i)];
        wait_berth_y = [serv_y(2),(berth_number-berth_pos(i)+1)*jam_spacing];
        
        if wait_berth(i) > 0 %??berth??????
            wait_berth_x = [end_serv_moment(i),end_serv_moment(i)+wait_berth(i)];
            wait_berth_y = [(berth_number-berth_pos(i)+1)*jam_spacing,(berth_number-berth_pos(i)+1)*jam_spacing];
        end
        line(wait_berth_x, wait_berth_y, 'linewidth',1);
        
        %4
        into_buffer_x = [wait_berth_x(2), wait_berth_x(2)+ fh*(buffer_number-u(i)+berth_pos(i))];
        into_buffer_y = [wait_berth_y(2), wait_berth_y(2)+jam_spacing*(buffer_number-u(i)+berth_pos(i))];
        line(into_buffer_x,into_buffer_y, 'linewidth',1);
        
        %5
        %wait_buffer_x = [into_buffer_x(2), leave_buffer_moment(i)];
        wait_buffer_x = [into_buffer_x(2), into_buffer_x(2)+wait_buffer(i)];
        wait_buffer_y = [into_buffer_y(2), into_buffer_y(2)];
        line(wait_buffer_x,wait_buffer_y,'linewidth',1);
        
        %6
        leave_buffer_x = [wait_buffer_x(2), wait_buffer_x(2)+3];
        leave_buffer_y = [wait_buffer_y(2), wait_buffer_y(2)+free_speed*3];
        line(leave_buffer_x,leave_buffer_y,'linewidth',1);
        
    end
end






