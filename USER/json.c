#include "json.h"

json_error_t *jsonerror;
json_t *jdata,*jsubdata,*jsubdata1;
char *json_out;

void jansson_init(void)
{
	json_set_alloc_funcs(json_malloc, json_free);
}

////创建温度曲线json文件
//static int CreateTemp_Jsonfile(const char *path)
//{
//	json_t *jarr,*jarr1,*jarr2,*jval;
//	u8 i,j;
//	int ret;

////	jdata = json_object();
//	jarr = json_array();
//	jdata = json_pack("{si}", /*"Enable", temp_data.HeatCoverEnable,*/"HeatCoverTemp", temp_data.HeatCoverTemp);
////	json_object_set(jdata,"HeatCover",jsubdata);

//	for(i=0;i<temp_data.StageNum;i++)	{
//		jarr2 = json_array();
//		jsubdata = json_pack("{si}", "Type",temp_data.stage[i].Type);
//		for(j=0;j<temp_data.stage[i].StepNum;j++)	{
//			jarr1 = json_array();
//			jval = json_integer( temp_data.stage[i].step[j].temp );
//			json_array_append_new( jarr1, jval );
//			jval = json_integer( temp_data.stage[i].step[j].tim );
//			json_array_append_new( jarr1, jval );
//			jval = json_integer( temp_data.stage[i].step[j].CollEnable );
//			json_array_append_new( jarr1, jval );
//			json_array_append_new( jarr2, jarr1 );
//		}
//		json_object_set_new(jsubdata, "Step", jarr2);
//		if(temp_data.stage[i].Type==StageByRepeat)
//			json_object_set_new(jsubdata, "Repeat", json_integer(temp_data.stage[i].RepeatNum));
//		else if(temp_data.stage[i].Type==StageByContinue)//溶解曲线
//			json_object_set_new(jsubdata, "T_Rate", json_integer(temp_data.stage[i].T_Rate));
//		else if(temp_data.stage[i].Type==StageByContinue)	{//溶解曲线
//			json_object_set_new(jsubdata, "T_Inter", json_integer(temp_data.stage[i].T_Inter));
//			json_object_set_new(jsubdata, "Const_Tim", json_integer(temp_data.stage[i].Const_Tim));
//		}
//		json_array_append_new( jarr, jsubdata );
//	}
//	json_object_set(jdata,"Stage",jarr);
//		
////	json_out = json_dumps(jdata, JSON_INDENT(1)|JSON_PRESERVE_ORDER|JSON_ENSURE_ASCII);
////	SYS_PRINTF("out:%s", json_out);
////	user_free(json_out);
//	ret = json_dump_file(jdata, path, JSON_INDENT(1)|JSON_PRESERVE_ORDER);
//	json_decref(jdata);
//	json_decref(jsubdata);

//	return ret;
//}

//创建lab json文件
int CreateLab_Jsonfile(const char *path)
{
	json_t *jarr,*jarr1,*jarr2,*jval;
	u8 i,j;
	int ret;
	
	jdata = json_object();	
	jarr = json_array();
	//实验属性
//	jsubdata = json_pack_ex(jsonerror,0,"{ss,ss,si,si,si}", "id", lab_data.id,"name",lab_data.name,"type",lab_data.type,"method",lab_data.method,"HoleNum",HOLE_NUM);
	jsubdata = json_pack("{ss,ss,si,si,si}", "id", lab_data.id,"name",lab_data.name,"type",lab_data.type,"method",lab_data.method,"HoleNum",HOLE_NUM);
	json_object_set(jdata,"Lab",jsubdata);
	//温度程序
	jsubdata1 = json_pack("{si}", "HeatCover", temp_data.HeatCoverTemp);
	for(i=0;i<temp_data.StageNum;i++)	{
		jarr2 = json_array();
		jsubdata = json_pack("{si}", "Type",temp_data.stage[i].Type);
		for(j=0;j<temp_data.stage[i].StepNum;j++)	{
			jarr1 = json_array();
			jval = json_integer( temp_data.stage[i].step[j].temp );
			json_array_append_new( jarr1, jval );
			jval = json_integer( temp_data.stage[i].step[j].tim );
			json_array_append_new( jarr1, jval );
			jval = json_integer( temp_data.stage[i].step[j].CollEnable );
			json_array_append_new( jarr1, jval );
			json_array_append_new( jarr2, jarr1 );
		}
		json_object_set_new(jsubdata, "Step", jarr2);
		if(temp_data.stage[i].Type==StageByRepeat)
			json_object_set_new(jsubdata, "Repeat", json_integer(temp_data.stage[i].RepeatNum));
		else if(temp_data.stage[i].Type==StageByContinue)//溶解曲线
			json_object_set_new(jsubdata, "T_Rate", json_integer(temp_data.stage[i].T_Rate));
		else if(temp_data.stage[i].Type==StageByContinue)	{//溶解曲线
			json_object_set_new(jsubdata, "T_Inter", json_integer(temp_data.stage[i].T_Inter));
			json_object_set_new(jsubdata, "Const_Tim", json_integer(temp_data.stage[i].Const_Tim));
		}
		json_array_append_new( jarr, jsubdata );
	}
	json_object_set_new(jsubdata1, "Stage",jarr);
	json_object_set(jdata,"Temp",jsubdata1);
	//样本信息
	for(i=0;i<HOLE_NUM;i++)	{
		if(sample_data.hole[i].sample_t[0] != 0)	
		{
			jsubdata = json_pack_ex(jsonerror,0,"{si,ss,ss}", "ID", i, "name",sample_data.hole[i].name, "prj",sample_data.hole[i].prj);	
			json_object_set_new(jsubdata, "sample", json_string((const char *)sample_data.hole[i].sample_t));
			json_object_set_new(jsubdata, "ch", json_string(sample_data.hole[i].channel));
			json_array_append_new( jarr, jsubdata );
		}
	}
	json_object_set(jdata,"Hole",jarr);
	ret = json_dump_file(jdata, path, JSON_INDENT(1)|JSON_PRESERVE_ORDER);
//	json_out = json_dumps(jdata, JSON_INDENT(1)|JSON_PRESERVE_ORDER|JSON_ENSURE_ASCII);
//	SYS_PRINTF("out:%s", json_out);
//	json_free(json_out);
	json_decref(jdata);
	json_decref(jsubdata);
	json_decref(jsubdata1);
	return ret;
}
//解析温度曲线json文件
//int AnalysisTemp_Jsonfile(const char *path)
//{
//	json_t *jtmp,*jtmp2,*jtmp3;
//	u8 i,j;

//	jdata = json_load_file(path, JSON_DECODE_ANY, jsonerror);
//	jsubdata = json_object_get((const json_t *)jdata,"HeatCoverTemp");
//	if(jsubdata!=NULL && json_is_object(jsubdata))	{
////		jtmp = json_object_get(jsubdata,"Enable");
////		temp_data.HeatCoverEnable = json_integer_value(jtmp);			
////		jtmp = json_object_get(jsubdata,"Temp");
//		temp_data.HeatCoverTemp = json_integer_value(jsubdata);			
//	}
//	json_decref(jsubdata);
//	jsubdata = json_object_get((const json_t *)jdata,"Stage");
//	if(jsubdata!=NULL && json_is_array(jsubdata))	{
//		temp_data.StageNum = json_array_size(jsubdata);
//		for(i=0;i<temp_data.StageNum;i++)	{
//			jtmp2 = json_array_get(jsubdata, i);
//			jtmp = json_object_get(jtmp2,"Type");
//			temp_data.stage[i].Type = json_integer_value(jtmp);					
//			jtmp3 = json_object_get(jtmp2,"Step");
//			temp_data.stage[i].StepNum = json_array_size(jtmp3);
//			for(j=0;j<temp_data.stage[i].StepNum;j++)	{
//				jtmp = json_array_get(jtmp3, j);
//				temp_data.stage[i].step[j].temp = json_integer_value(json_array_get(jtmp, 0));
//				temp_data.stage[i].step[j].tim = json_integer_value(json_array_get(jtmp, 1));
//				temp_data.stage[i].step[j].CollEnable = json_integer_value(json_array_get(jtmp, 2));
//				json_decref(jtmp);
//			}
//			
//			if(temp_data.stage[i].Type==StageByRepeat)	{
//				temp_data.stage[i].RepeatNum = json_integer_value(json_object_get(jtmp2,"Repeat"));
//			}
//			else if(temp_data.stage[i].Type==StageByContinue)	{//溶解曲线
//				temp_data.stage[i].T_Rate = json_integer_value(json_object_get(jtmp2,"T_Rate"));
//			}
//			else if(temp_data.stage[i].Type==StageByContinue)	{//溶解曲线
//				temp_data.stage[i].T_Inter = json_integer_value(json_object_get(jtmp2,"T_Inter"));
//				temp_data.stage[i].Const_Tim = json_integer_value(json_object_get(jtmp2,"Const_Tim"));
//			}
//		}
//		json_decref(jdata);
//		json_decref(jsubdata);
//		return 0;
//	}
//	return -1;
//}

//解析温lab json文件
int AnalysisLab_Jsonfile(const char *path)
{
	json_t *jtmp=NULL,*jtmp2=NULL,*jtmp3=NULL;
	u8 i,j;
	u16 total;

	jdata = json_load_file(path, JSON_DECODE_ANY, jsonerror);
	//解析实验属性
	jsubdata = json_object_get((const json_t *)jdata,"Lab");
	if(jsubdata!=NULL && json_is_object(jsubdata))	{
		strcpy(lab_data.id, json_string_value(json_object_get(jsubdata,"id")));
		strcpy(lab_data.name, json_string_value(json_object_get(jsubdata,"name")));
		lab_data.type = json_integer_value(json_object_get(jsubdata,"type"));
		lab_data.method = json_integer_value(json_object_get(jsubdata,"method"));
//		lab_data. = json_integer_value(json_object_get(jsubdata,"HoleNum"));
	}
	json_decref(jsubdata);
	//解析温度程序
	jsubdata1 = json_object_get(jdata,"Temp");
	if(jsubdata1!=NULL && json_is_object(jsubdata1))	{
		jsubdata = json_object_get((const json_t *)jsubdata1,"HeatCover");
		if(jsubdata!=NULL && json_is_object(jsubdata))	{
			temp_data.HeatCoverTemp = json_integer_value(jsubdata);			
		}
		json_decref(jsubdata);
		jsubdata = json_object_get((const json_t *)jsubdata1,"Stage");
		if(jsubdata!=NULL && json_is_array(jsubdata))	{
			temp_data.StageNum = json_array_size(jsubdata);
			for(i=0;i<temp_data.StageNum;i++)	{
				jtmp2 = json_array_get(jsubdata, i);
				jtmp = json_object_get(jtmp2,"Type");
				temp_data.stage[i].Type = json_integer_value(jtmp);					
				jtmp3 = json_object_get(jtmp2,"Step");
				temp_data.stage[i].StepNum = json_array_size(jtmp3);
				for(j=0;j<temp_data.stage[i].StepNum;j++)	{
					jtmp = json_array_get(jtmp3, j);
					temp_data.stage[i].step[j].temp = json_integer_value(json_array_get(jtmp, 0));
					temp_data.stage[i].step[j].tim = json_integer_value(json_array_get(jtmp, 1));
					temp_data.stage[i].step[j].CollEnable = json_integer_value(json_array_get(jtmp, 2));
					json_decref(jtmp);
				}
				
				if(temp_data.stage[i].Type==StageByRepeat)	{
					temp_data.stage[i].RepeatNum = json_integer_value(json_object_get(jtmp2,"Repeat"));
				}
				else if(temp_data.stage[i].Type==StageByContinue)	{//溶解曲线
					temp_data.stage[i].T_Rate = json_integer_value(json_object_get(jtmp2,"T_Rate"));
				}
				else if(temp_data.stage[i].Type==StageByContinue)	{//溶解曲线
					temp_data.stage[i].T_Inter = json_integer_value(json_object_get(jtmp2,"T_Inter"));
					temp_data.stage[i].Const_Tim = json_integer_value(json_object_get(jtmp2,"Const_Tim"));
				}
			}
		}
		json_decref(jsubdata);		
	}
	json_decref(jsubdata1);
	//解析样本信息
	jsubdata = json_object_get(jdata,"Hole");
	total = json_array_size(jsubdata);
	for(i=0;i<total;i++)	{
		jtmp = json_array_get(jsubdata, i);
		j = json_integer_value(json_object_get(jtmp,"ID"));
		if(j<HOLE_NUM)	{
			strcpy(sample_data.hole[j].name, json_string_value(json_object_get(jtmp,"name")));
			strcpy(sample_data.hole[j].prj, json_string_value(json_object_get(jtmp,"prj")));
			strcpy(sample_data.hole[j].sample_t, json_string_value(json_object_get(jtmp,"sample")));			
			strcpy(sample_data.hole[j].channel, json_string_value(json_object_get(jtmp,"ch")));
		}
		json_decref(jtmp);
	}
	json_decref(jdata);
	json_decref(jsubdata);
	return 0;
}

//添加二维数组到json文件
void add_2array_to_json( json_t* obj, const char* name, const int* arr_adrr, size_t row, size_t column)
{
	size_t i, j;
	json_t *jarr1,*jarr2;
	json_t* jval;
	int val;
	
	jarr1 = json_array();
	for( i=0; i<row; ++i ) {
		jarr2 = json_array();
		for( j=0; j<column; ++j ) {
			val = arr_adrr[ i*column + j ];
			jval = json_integer( val );
			json_array_append_new( jarr2, jval );
		}
		json_array_append_new( jarr1, jarr2 );
	}
	json_object_set_new( obj, name, jarr1 );
}











#if 1
json_t *root;
//jansson Test
int arr1[2][3] = { {1,2,3}, {4,5,6} };
int arr2[4][4] = { {1,2,3,4}, {5,6,7,8}, {9,10,11,12}, {13,14,15,16} };
void jansson_pack_test(void)
{
	/* Build an empty JSON object */
//	root = json_pack("{}");
//	
//	json_out = json_dumps(root, JSON_ENCODE_ANY);
//	SYS_PRINTF("out:%s", json_out);
//	json_decref(root);
//	user_free(json_out);
	
	/* Build the JSON object {"foo": 42, "bar": 7} */
//	root = json_pack("{sisi}", "foo", 42, "bar", 7);
	root = json_pack_ex(jsonerror,0,"{sisi}", "foo", 42, "bar", 5);

	json_out = json_dumps(root, JSON_INDENT(1)|JSON_PRESERVE_ORDER);
	SYS_PRINTF("out:%s", json_out);
	json_decref(root);
	user_free(json_out);
	
	/* Like above, ':', ',' and whitespace are ignored */
	root = json_pack("{s:i, s:i}", "foo", 42, "bar", 7);

	json_out = json_dumps(root, JSON_ENCODE_ANY);
	SYS_PRINTF("out:%s", json_out);
	json_decref(root);
	user_free(json_out);
	
	/* Build the JSON array [[1, 2], {"cool": true}] */
	root = json_pack("[[i,i],{s:b}]", 1, 2, "cool", 1);

	json_out = json_dumps(root, JSON_ENCODE_ANY);
	SYS_PRINTF("out:%s", json_out);
	json_decref(root);
	user_free(json_out);
	
	/* Build a string from a non-null terminated buffer */
	char buffer[4] = {'t', 'e', 's', 't'};
	root = json_pack("[s#]", buffer, 4);

	json_out = json_dumps(root, JSON_ENCODE_ANY);
	SYS_PRINTF("out:%s", json_out);
	json_decref(root);
	user_free(json_out);
	
	/* Concatenate strings together to build the JSON string "foobarbaz" */
	root = json_pack("[s++]", "foo", "bar", "baz");
	
	json_out = json_dumps(root, JSON_ENCODE_ANY);
	SYS_PRINTF("out:%s", json_out);
	json_decref(root);
	user_free(json_out);
//out:{"arr1": [[1, 2, 3], [4, 5, 6]], "arr2": [[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12], [13, 14, 15, 16]]}
	root = json_object();
	add_2array_to_json( root, "arr1", &arr1[0][0], 2, 3 );
	add_2array_to_json( root, "arr2", &arr2[0][0], 4, 4 );
	json_out = json_dumps( root, JSON_ENCODE_ANY );
	SYS_PRINTF("out:%s", json_out);
	json_decref(root);
	user_free(json_out);
}
#endif
