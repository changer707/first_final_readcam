#ifndef JUDGE_H
#define JUDGE_H
#include "missingbox.h"
#include "mybox.h"
#include "distance.h"
#include "getline.h"
#include <iostream>





/*1.判断前后帧之间是否有ID消失，若有，返回ID，type,frame;*/
std::vector<missingbox> RXY_findmissingbox(std::vector<mybox>& detections1,std::vector<mybox>& detections2){
     //missingbox miss_boxes;
    std::vector<missingbox> miss_boxes;//all_detection[i]中消失的所有对象
    //all_detection[i]与all_detection[i+1]之间的帧数差
   int delta_frame=abs(detections1.at(0).frame-detections2.at(0).frame);
    //帧数差大于1,说明中间有无物体状态
if(delta_frame>1){
    std::vector<mybox>::iterator it_det1;
    for(it_det1=detections1.begin();it_det1<detections1.end();++it_det1){
       missingbox missbox;
       float dxn;float dyn;
       missbox.x=it_det1->boxings.x;
       missbox.y=it_det1->boxings.y;
       missbox.vx=it_det1->vx;
       missbox.vy=it_det1->vy;
       missbox.dx=missbox.vx*delta_T;
       missbox.dy=missbox.vy*delta_T;
       missbox.wideth=it_det1->boxings.width;
       missbox.height=it_det1->boxings.height;
       missbox.missing_area=it_det1->boxings.area();
       missbox.missing_ID=it_det1->ID_number;
       missbox.missing_type=it_det1->typenumber;
       missbox.missing_frame=it_det1->frame;

       dxn=fabs(missbox.vx*delta_T);
       dyn=fabs(missbox.vy*delta_T);
       if(missbox.dx){
           missbox.k=dyn/dxn;
       }
       else{
           missbox.k=infinity_double;
       }

       miss_boxes.push_back(missbox);
    }
}
else{
    bool flag_findid=true;
//    std::cout<<detections1.empty()<<std::endl;
//    std::cout<<detections1.size()<<std::endl;
//    std::cout<<detections1.at(0).ID_number<<detections1.at(0).boxings.x<<std::endl;
//    std::cout<<detections1.at(3).ID_number<<detections1.at(3).boxings.x<<std::endl;
//    std::cout<<detections1.at(10).ID_number<<detections1.at(10).boxings.x<<std::endl;

    for(size_t i=0;i<detections1.size();i++){
        flag_findid=false;float dx=0;float dy=0;float dxm=0;float dym=0;
//        std::cout<<2222<<std::endl;

        for(size_t j=0;j<detections2.size();j++){
           if(detections1.at(i).ID_number==detections2.at(j).ID_number)//未消失
           {
               dx=fabs(detections1.at(i).boxings.x-detections2.at(j).boxings.x);
               dy=fabs(detections1.at(i).boxings.y-detections2.at(j).boxings.y);
               if(dx!=0){
                   detections1.at(i).k=dy/dx;//运动方向的比例系数
               }
               else {
                   detections1.at(i).k=infinity_double;//;//浮点数表示的正无穷大
               }
               detections1.at(i).dx=detections2.at(j).boxings.x-detections1.at(i).boxings.x;
               detections1.at(i).dy=detections2.at(j).boxings.y-detections1.at(i).boxings.y;
               flag_findid=true;
               break;
           }
        }
        if(!flag_findid){
            //记录ID消失前一帧的信息
             missingbox miss_box;

             miss_box.x=detections1.at(i).boxings.x;
             miss_box.y=detections1.at(i).boxings.y;
             miss_box.vx=detections1.at(i).vx;//有正负
             miss_box.vy=detections1.at(i).vy;
             miss_box.dx=miss_box.vx*delta_T;//有正负
             miss_box.dy=miss_box.vy*delta_T;
             miss_box.wideth=detections1.at(i).boxings.width;
             miss_box.height=detections1.at(i).boxings.height;
             miss_box.missing_area=detections1.at(i).boxings.area();
             miss_box.missing_ID=detections1.at(i).ID_number;
             miss_box.missing_type=detections1.at(i).typenumber;
             miss_box.missing_frame=detections1.at(0).frame;//消失前一帧

             dxm=fabs(miss_box.vx*delta_T);
             dym=fabs(miss_box.vy*delta_T);
             if(miss_box.dx){
                 miss_box.k=dym/dxm;
             }
             else{
                 miss_box.k=infinity_double;
             }

             miss_boxes.push_back(miss_box);
        }
    }
  }
    return miss_boxes;
}

/*3.寻找消失ID周围r区域内的对象*/
std::vector<mybox> RXY_findneighbour(missingbox&miss_box,const std::vector<mybox>&detections){
    std::vector<mybox> neighbourboxes;
    std::vector<mybox>::const_iterator it_detections;
    for(it_detections=detections.begin();it_detections<detections.end();it_detections++){
        float dx=fabs((*it_detections).boxings.x-miss_box.x);
        float dy=fabs((*it_detections).boxings.y-miss_box.y);
        float dist=sqrt(dx*dx+dy*dy);
        std::cout<<dist<<",";
        //不把miss_box本身包括进neighbour
        if(miss_box.missing_ID!=it_detections->ID_number){
            std::cout<<r_threshold<<std::endl;
//            if(it_detections->typenumber<6){
                if(dist<r_threshold) {neighbourboxes.push_back(*it_detections);}
                //如果图面中有货车和客车,增大查找范围1.5倍
                else {
                    if(it_detections->typenumber>=6){
                        if(dist<1.5*r_threshold) {neighbourboxes.push_back(*it_detections);}
                    }
                }
//            }
            //遇到truck,heavytruck,coach时neighbour的搜索范围大一些
//            else {
//                if(dist<r_big_threshold) {neighbourboxes.push_back(*it_detections);}
//            }
        }
    }
    std::cout<<neighbourboxes.size()<<std::endl;
    return neighbourboxes;
}


/*4.对于疑似消失的对象，在后10帧中做检测，检查是否出现，出现坐标和原坐标是否有较大的变动*/
void RXY_morecheck(const std::vector<mybox>&detections,std::vector<missingbox>&like_accidentboxes,std::vector<missingbox>&miss_to_accidentboxes){
    //    std::vector<missingbox> miss_to_perframe;
    std::vector<missingbox>::iterator it_like_acc;
    std::vector<mybox>::const_iterator it_det;
    //    bool flag_match=false;
    for(it_like_acc=like_accidentboxes.begin();it_like_acc<like_accidentboxes.end();++it_like_acc){
        it_like_acc->flag_merge=false;
        if(it_like_acc->like_check>like_check_thre){continue;}
        else if(it_like_acc->like_check==like_check_thre){it_like_acc->like_check++;miss_to_accidentboxes.push_back(*it_like_acc);}
        else{
            //超过计数阈值noacci_count_thre的对象认为未出事故,不再执行后续工作
            if(it_like_acc->noacci_count<noacci_count_thre){

                for(it_det=detections.begin();it_det<detections.end();++it_det){
                    //若疑似对象出现
                    if(it_like_acc->missing_ID==it_det->ID_number){
                        it_like_acc->flag_merge=true;
                        float DX=fabs(it_det->boxings.x-it_like_acc->x);
                        float DY=fabs(it_det->boxings.y-it_like_acc->y);
                        float D_sum=sqrt(DX*DX+DY*DY);
                        //???可能会出现被遮挡只框出露出的一部分,识别框的中心点移动的情况
                        //r_move应大于框的一半宽度
                        if((D_sum>(it_like_acc->wideth)*kmove)||it_like_acc->v_sum>20){
                            it_like_acc->flag_accident=false;
                            //计数,防止非事故对象的累积
                            it_like_acc->noacci_count++;
                            /*it_like_acc=like_accidentboxes.erase(it_like_acc);*///只挑出出事故的对象，对未出事故的不关心
                            break;
                        }
                        else{
                            if(it_like_acc->like_check<like_check_thre)//疑似事故对象检测帧
                                //{miss_to_accidentboxes.push_back(*it_like_acc);}
                                //可能速度比较慢,只检测当like_check结束以后的位置
                            {(*it_like_acc).like_check++;}
                            else{miss_to_accidentboxes.push_back(*it_like_acc);}//如果超过检查次数，则判为发生事故
                            break;
                        }
                    }
                }

                if(!it_like_acc->flag_merge){
                    if(it_like_acc->like_check<like_check_thre){(*it_like_acc).like_check++;}
                    else {
                        miss_to_accidentboxes.push_back(*it_like_acc);}
                }
            }
        }
    }
    //    return miss_to_accidentboxes;
}



std::vector<mybox> RXY_findneighbour1(mybox&nomiss_box,const std::vector<mybox>&detections){
    std::vector<mybox> neighbourboxes;
    std::vector<mybox>::const_iterator it_detections;
    for(it_detections=detections.begin();it_detections<detections.end();it_detections++){
        float dx=fabs((*it_detections).boxings.x-nomiss_box.boxings.x);
        float dy=fabs((*it_detections).boxings.y-nomiss_box.boxings.y);
        float dist=sqrt(dx*dx+dy*dy);
        //不把miss_box本身包括进neighbour
        if(nomiss_box.ID_number!=it_detections->ID_number){
//            if(it_detections->typenumber<6){
                if(dist<r_threshold) {neighbourboxes.push_back(*it_detections);}
//            }
            //遇到truck,heavytruck,coach时neighbour的搜索范围大一些
//            else {
//                if(dist<r_big_threshold) {neighbourboxes.push_back(*it_detections);}
//            }
        }
    }
    return neighbourboxes;
}



void RXY_neighcheck_accident(const std::vector<mybox>&detections,std::vector<missingbox>&miss_to_accident,std::vector<missingbox>&fmiss_to_accident){
     std::vector<missingbox>::iterator it_miss;
     for(it_miss=miss_to_accident.begin();it_miss<miss_to_accident.end();++it_miss){
         it_miss->crashone.neigh_check++;
         if(it_miss->crashone.neigh_check>=neigh_check_thre){
             if(!it_miss->crashone.flag_neighmove&&(!it_miss->crashone.flag_added)){
                 fmiss_to_accident.push_back(*it_miss);
                 it_miss->crashone.flag_added=true;
             }
             else {
                 continue;
             }
         }
         else{
             if(!it_miss->crashone.flag_neighmove){
                 for(int i=0;i<detections.size();i++){
                    //检测crashone是否出现
                     if(it_miss->crashone.ID_number==detections[i].ID_number){
                         it_miss->crashone.flag_merge_neighstay1=true;
                         float dx=fabs(detections[i].boxings.x-it_miss->crashone.boxings.x);
                         float dy=fabs(detections[i].boxings.y-it_miss->crashone.boxings.y);
                         float dsum=sqrt(dx*dx+dy*dy);
                         it_miss->crashone.dist_neighcheck=dsum;
                    //移动是否大于0.1*wideth
                         if(dsum>(it_miss->crashone.boxings.width*nmove)){
                            it_miss->crashone.flag_neighmove=true;
                         }
                         break;
                     }
                 }
             }
         }
     }
}


//miss,crashone最高速者vsum_filter小于300
//考虑对于这种类型的miss_to_accident,检测其crashone在100帧后是否仍然存在且速度趋近于0,坐标不再移动
void RXY_crashonestay(const std::vector<mybox>&detections,std::vector<missingbox>&miss_to_accident,std::vector<missingbox>&stay_to_accident){
    std::vector<missingbox>::iterator it_miss;
    for(it_miss=miss_to_accident.begin();it_miss<miss_to_accident.end();++it_miss){
        it_miss->crashone.stay_check++;
        if(it_miss->crashone.stay_check>=stay_checkthre){continue;}//100帧里没有发现停下来,就不认为事故
        else {
            if(!it_miss->crashone.flag_stay){
                for(int i=0;i<detections.size();i++){
                    //检测crashone是否出现
                    if(it_miss->crashone.ID_number==detections[i].ID_number){
                        it_miss->crashone.flag_merge_stay1=true;
                        //速度趋近于0
                        if(detections[i].v_sum<vsum_staythre){
                            //记录每一帧已滑行距离
                            float dx=fabs(it_miss->crashone.boxings.x-detections[i].boxings.x);
                            float dy=fabs(it_miss->crashone.boxings.y-detections[i].boxings.y);
                            float dsum=sqrt(dx*dx+dy*dy);
                            if(!it_miss->crashone.dist_slide.empty()){
                                float delta_slide_dist=fabs(dsum-it_miss->crashone.dist_slide.back());
                                //坐标不再移动
                                if(delta_slide_dist<slide_staythre){
                                    it_miss->crashone.flag_stay=true;
                                    stay_to_accident.push_back(*it_miss);
                                }
                            }
                            it_miss->crashone.dist_slide.push_back(dsum);
                        }
                    }
                }
            }
        }
    }
}

//miss,crashone最高速者vsum_filter小于300
//考虑对于这种类型的miss_to_accident,检测其miss在100帧后是否仍然存在且速度趋近于0,坐标不再移动
void RXY_missstay(const std::vector<mybox>&detections,std::vector<missingbox>&miss_to_accident,std::vector<missingbox>&stay_to_accident){
    std::vector<missingbox>::iterator it_miss;
    for(it_miss=miss_to_accident.begin();it_miss<miss_to_accident.end();++it_miss){
        it_miss->stay_check++;
        if(it_miss->stay_check>=stay_checkthre){continue;}//100帧里没有发现停下来,就不认为事故
        else {
            if(!it_miss->flag_stay){
                for(int i=0;i<detections.size();i++){
                    //检测crashone是否出现
                    if(it_miss->missing_ID==detections[i].ID_number){
                        it_miss->flag_merge_stay1=true;
                        //速度趋近于0
                        if(detections[i].v_sum<vsum_staythre){
                            //记录每一帧已滑行距离
                            float dx=fabs(it_miss->x-detections[i].boxings.x);
                            float dy=fabs(it_miss->y-detections[i].boxings.y);
                            float dsum=sqrt(dx*dx+dy*dy);
                            if(!it_miss->dist_slide.empty()){
                                float delta_slide_dist=fabs(dsum-it_miss->dist_slide.back());
                                //坐标不再移动
                                if(delta_slide_dist<slide_staythre){
                                    it_miss->flag_stay=true;
                                    stay_to_accident.push_back(*it_miss);
                                }
                            }
                            it_miss->dist_slide.push_back(dsum);
                        }
                    }
                }
            }
        }
    }
}




//miss,crashone最高速者vsum_filter小于300
//考虑对于这种类型的miss_to_accident,检测其crashone在100帧后是否仍然存在且速度趋近于0,坐标不再移动
void RXY_crashonestay1(const std::vector<mybox>&detections,std::vector<mybox>&nomiss_to_accident,std::vector<mybox>&stay_to_accident){
    std::vector<mybox>::iterator it_nomiss;
    for(it_nomiss=nomiss_to_accident.begin();it_nomiss<nomiss_to_accident.end();++it_nomiss){
        it_nomiss->crashone1.stay_check++;
        if(it_nomiss->crashone1.stay_check>=stay_checkthre){continue;}//100帧里没有发现停下来,就不认为事故
        else {
            if(!it_nomiss->crashone1.flag_stay){
                for(int i=0;i<detections.size();i++){
                    //检测crashone是否出现
                    if(it_nomiss->crashone1.ID_number==detections[i].ID_number){
                        it_nomiss->crashone1.flag_merge_stay1=true;
                        //速度趋近于0
                        if(detections[i].v_sum<vsum_staythre){
                            //记录每一帧已滑行距离
                            float dx=fabs(it_nomiss->crashone1.boxings.x-detections[i].boxings.x);
                            float dy=fabs(it_nomiss->crashone1.boxings.y-detections[i].boxings.y);
                            float dsum=sqrt(dx*dx+dy*dy);
                            if(!it_nomiss->crashone1.dist_slide.empty()){
                                float delta_slide_dist=fabs(dsum-it_nomiss->crashone1.dist_slide.back());
                                //坐标不再移动
                                if(delta_slide_dist<slide_staythre){
                                    it_nomiss->crashone1.flag_stay=true;
                                    stay_to_accident.push_back(*it_nomiss);
                                }
                            }
                            it_nomiss->crashone1.dist_slide.push_back(dsum);
                        }
                    }
                }
            }
        }
    }
}

//miss,crashone最高速者vsum_filter小于300
//考虑对于这种类型的miss_to_accident,检测其miss在100帧后是否仍然存在且速度趋近于0,坐标不再移动
void RXY_nomissstay(const std::vector<mybox>&detections,std::vector<mybox>&nomiss_to_accident,std::vector<mybox>&stay_to_accident){
    std::vector<mybox>::iterator it_nomiss;
    for(it_nomiss=nomiss_to_accident.begin();it_nomiss<nomiss_to_accident.end();++it_nomiss){
        it_nomiss->stay_check++;
        if(it_nomiss->stay_check>=stay_checkthre){continue;}//100帧里没有发现停下来,就不认为事故
        else {
            if(!it_nomiss->flag_stay){
                for(int i=0;i<detections.size();i++){
                    //检测crashone是否出现
                    if(it_nomiss->ID_number==detections[i].ID_number){
                        it_nomiss->flag_merge_stay1=true;
                        //速度趋近于0
                        if(detections[i].v_sum<nomiss_vsum_staythre){
                            //记录每一帧已滑行距离
                            float dx=fabs(it_nomiss->boxings.x-detections[i].boxings.x);
                            float dy=fabs(it_nomiss->boxings.y-detections[i].boxings.y);
                            float dsum=sqrt(dx*dx+dy*dy);
                            if(!it_nomiss->dist_slide.empty()){
                                float delta_slide_dist=fabs(dsum-it_nomiss->dist_slide.back());
                                //坐标不再移动
                                if(delta_slide_dist<nomiss_slide_staythre){
                                    it_nomiss->flag_stay=true;
                                    stay_to_accident.push_back(*it_nomiss);
                                }
                            }
                            it_nomiss->dist_slide.push_back(dsum);
                        }
                    }
                }
            }
        }
    }
}

//速度修正,对vy&vx进行修正,vsum_filter
void RXY_Vfix_faccident(std::vector<missingbox>& fmiss_to_accident){
     std::vector<missingbox>::iterator it_fmiss;
     //y方向速度修正比例系数, y=520处l_filter=1
     for(it_fmiss=fmiss_to_accident.begin();it_fmiss<fmiss_to_accident.end();++it_fmiss){
         if(it_fmiss->y<520){
             it_fmiss->k_vfilter=0.1055;
             it_fmiss->b_vfilter=-25.9025;//-53.86;
             it_fmiss->crashone.k_vfilter=0.1055;
             it_fmiss->crashone.b_vfilter=-25.9025;//-53.86;
             //修正系数
             it_fmiss->l_vfilter=(it_fmiss->k_vfilter)*(it_fmiss->y)+it_fmiss->b_vfilter;
             it_fmiss->vy_filter=vyf_gain*(it_fmiss->vy)/(it_fmiss->l_vfilter);
//             it_fmiss->vx_filter=(it_fmiss->vx)/(it_fmiss->l_vfilter);
             it_fmiss->vsum_filter=sqrt(it_fmiss->vy_filter*it_fmiss->vy_filter+it_fmiss->vx*it_fmiss->vx);

             it_fmiss->crashone.l_vfilter=(it_fmiss->crashone.k_vfilter)*(it_fmiss->crashone.boxings.y)+it_fmiss->crashone.b_vfilter;
             it_fmiss->crashone.vy_filter=vyf_gain*(it_fmiss->crashone.vy)/(it_fmiss->crashone.l_vfilter);
//             it_fmiss->crashone.vx_filter=(it_fmiss->crashone.vx)/(it_fmiss->crashone.l_vfilter);
             it_fmiss->crashone.vsum_filter=sqrt(it_fmiss->crashone.vy_filter*it_fmiss->crashone.vy_filter+it_fmiss->crashone.vx*it_fmiss->crashone.vx);
         }
         else {
             it_fmiss->k_vfilter=0.0979;
             it_fmiss->b_vfilter=-72.9635;//-49.908;
             it_fmiss->crashone.k_vfilter=0.0979;
             it_fmiss->crashone.b_vfilter=-72.9635;//-49.908;

             it_fmiss->l_vfilter=(it_fmiss->k_vfilter)*(it_fmiss->y)+it_fmiss->b_vfilter;
             it_fmiss->vy_filter=vyf_gain*(it_fmiss->vy)/(it_fmiss->l_vfilter);
//             it_fmiss->vx_filter=(it_fmiss->vx)/(it_fmiss->l_vfilter);
             it_fmiss->vsum_filter=sqrt(it_fmiss->vy_filter*it_fmiss->vy_filter+it_fmiss->vx*it_fmiss->vx);

             it_fmiss->crashone.l_vfilter=(it_fmiss->crashone.k_vfilter)*(it_fmiss->crashone.boxings.y)+it_fmiss->crashone.b_vfilter;
             it_fmiss->crashone.vy_filter=vyf_gain*(it_fmiss->crashone.vy)/(it_fmiss->crashone.l_vfilter);
//             it_fmiss->crashone.vx_filter=(it_fmiss->crashone.vx)/(it_fmiss->crashone.l_vfilter);
             it_fmiss->crashone.vsum_filter=sqrt(it_fmiss->crashone.vy_filter*it_fmiss->crashone.vy_filter+it_fmiss->crashone.vx*it_fmiss->crashone.vx);
         }

     }

}

//速度滤波,将低速的fmiss_to_accident去掉;
//若miss_to_accident的速度小,而crashone的速度大,也认为是发生了事故(300>速度"小":>100;速度"大":>300)
void RXY_Vfilter_faccident(std::vector<missingbox>& miss_to_accident,std::vector<missingbox>& vfilter_fmiss,std::vector<missingbox>& vfilter_for_stay){
    std::vector<missingbox>::iterator it_miss;
    for(it_miss=miss_to_accident.begin();it_miss<miss_to_accident.end();++it_miss){
        //(1) (miss||crashone) >300
        if((it_miss->vsum_filter>vfilter_thre||it_miss->crashone.vsum_filter>vfilter_thre)&&(!it_miss->flag_fvout)){
            //(2) (miss&&crashone) >20
            if(it_miss->vsum_filter>vfilter_lowthre&&it_miss->crashone.vsum_filter>vfilter_lowthre){
            vfilter_fmiss.push_back(*it_miss);
            it_miss->flag_fvout=true;//防止同一对象重复放入vfilter_fmiss数组
            }
            //(3) miss为y向移动物体:vy_filter本来就小,vx小,vsum_filter小
            //    (i) crashone>20  (ii) |vyf| > vyf_thre
            if((!it_miss->flag_fvout)&&it_miss->crashone.vsum_filter>vfilter_lowthre&&fabs(it_miss->vy_filter)>vy_filter_thre){
            vfilter_fmiss.push_back(*it_miss);
            it_miss->flag_fvout=true;
            }
        }

        //(1)若miss和crashone都小于300,但其实速度还是还是比较大的,这种,放入数组,用stay筛选
        else{
            //(1) (miss||crashone) >30
            if((it_miss->vsum_filter>vsumfilter_forstay_thre||it_miss->crashone.vsum_filter>vsumfilter_forstay_thre)&&(!it_miss->flag_fvout)){
                //(2) (miss&&crashone) >15
                if(it_miss->vsum_filter>vfstay_lowstay&&it_miss->crashone.vsum_filter>vfstay_lowstay){
                vfilter_for_stay.push_back(*it_miss);
                it_miss->flag_fvout=true;//防止同一对象重复放入vfilter_for_stay数组
                }
                //(3) miss为y向移动物体:vy_filter本来就小,vx小,vsum_filter小
                //    (i) crashone>20  (ii) |vyf| > vyf_thre
                if((!it_miss->flag_fvout)&&it_miss->crashone.vsum_filter>vfstay_lowstay&&fabs(it_miss->vy_filter)>vy_filter_thre){
                vfilter_for_stay.push_back(*it_miss);
                it_miss->flag_fvout=true;
                }
            }
        }
    }
}


//寻找a和v都很大的物体
void RXY_highacceleration(const std::vector<mybox>& detections,std::vector<mybox>& high_accelerationboxes,std::vector<motionparasbox>& motionparasboxes){
    std::vector<mybox>::const_iterator it_det;
    std::vector<motionparasbox>::iterator it_paras;
    bool flag=false;
    for(it_det=detections.begin();it_det<detections.end();++it_det){
        if(it_det->a_sum>asum_thre){
            if(it_det->v_sum>vsum_thre){
                if(!motionparasboxes.empty()){
                    flag=false;
                    for(it_paras=motionparasboxes.begin();it_paras<motionparasboxes.end();++it_paras){
                        if(it_det->ID_number==it_paras->id){
                            flag=true;
                            if(it_det->a_sum>it_paras->asum&&it_det->v_sum>it_paras->vsum){
                                it_paras->asum=it_det->a_sum;
                                it_paras->vsum=it_det->v_sum;
                                std::vector<mybox>::iterator it_high;
                                for(it_high=high_accelerationboxes.begin();it_high<high_accelerationboxes.end();++it_high){
                                    if(it_high->ID_number==it_det->ID_number){
                                        *it_high=*it_det;
                                    }
                                }
                            }
                        }
                    }
                    if(!flag){
                        high_accelerationboxes.push_back(*it_det);
                        motionparasbox box1;
                        box1.id=it_det->ID_number;
                        box1.asum=it_det->a_sum;
                        box1.vsum=it_det->v_sum;
                        box1.flagadd_paras=true;
                        motionparasboxes.push_back(box1);
                    }
                }
                else{
                    high_accelerationboxes.push_back(*it_det);
                    motionparasbox box;
                    box.id=it_det->ID_number;
                    box.asum=it_det->a_sum;
                    box.vsum=it_det->v_sum;
                    box.flagadd_paras=true;
                    motionparasboxes.push_back(box);
                }
            }
        }
    }
}


#endif // JUDGE_H
