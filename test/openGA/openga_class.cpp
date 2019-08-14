//
// Created by wang on 19-6-4.
//

#include "openga_class.h"
void openga_class::solve(){
    output_file.open("../bin/result_so1.txt");
    output_file<<"step"<<"\t"<<"x_best"<<"\t"<<"y_best"<<"\t"<<"cost_avg"<<"\t"<<"cost_best"<<"\n";

    EA::Chronometer timer;
    timer.tic();


    ga_obj.problem_mode= EA::GA_MODE::SOGA;
    ga_obj.multi_threading=true;
    ga_obj.idle_delay_us=1; // switch between threads quickly
    ga_obj.verbose=true;
    ga_obj.population=20;
    ga_obj.generation_max=1000;
    ga_obj.calculate_SO_total_fitness=calculate_SO_total_fitness;
    ga_obj.init_genes= init_genes;
    ga_obj.eval_solution= eval_solution;
    ga_obj.mutate= mutate;
    ga_obj.crossover= crossover;
    ga_obj.SO_report_generation= SO_report_generation;
    ga_obj.best_stall_max=10;
    ga_obj.elite_count=10;
    ga_obj.crossover_fraction=0.7;
    ga_obj.mutation_rate=0.4;
    ga_obj.solve();

    std::cout<<"The problem is optimized in "<<timer.toc()<<" seconds."<<std::endl;

    output_file.close();
    auto best_chromosomes = ga_obj.last_generation.chromosomes[ga_obj.last_generation.best_chromosome_index];
    std::cout<< "best chromosomes = ("<<best_chromosomes.genes.x<<","<<best_chromosomes.genes.y<<")"<<std::endl;
}
std::ofstream openga_class::output_file;