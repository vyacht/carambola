#include <string.h>
#include <jansson.h>


char * read() {
  FILE *fp;
  long lSize;
  char *buffer;

  fp = fopen ( "/tmp/signalk.json" , "rb" );
  if( !fp ) perror("blah.txt"),exit(1);

  fseek( fp , 0L , SEEK_END);
  lSize = ftell( fp );
  rewind( fp );

  /* allocate memory for entire content */
  buffer = calloc( 1, lSize+1 );
  if( !buffer ) fclose(fp),fputs("memory alloc fails",stderr),exit(1);

  /* copy the file into the buffer */
  if( 1!=fread( buffer , lSize, 1 , fp) )
  fclose(fp),free(buffer),fputs("entire read fails",stderr),exit(1);

  /* do your work here, buffer is a string contains the whole text */

  fclose(fp);

  return buffer;

}

int main(int argc, char *argv[])
{
    size_t i;
    char *text;

    json_t *root, *vessels;
    json_error_t error;

    text = read();
    root = json_loads(text, 0, &error);
    free(text);

    if(!root)
    {
       fprintf(stderr, "error: on line %d: %s\n", error.line, error.text);
       return 1;
    }

    json_t *data, *navigation;
    const char *message_text;

//    data = json_array_get(root, i);
    vessels = json_object_get(root, "vessels");
    if(!json_is_array(vessels))
    {
        fprintf(stderr, "error: commit data %s is not an array\n", "vessel");
        json_decref(root);
        return 1;
    }

    for(i = 0; i < json_array_size(vessels); i++) {
      printf("[%d]\n", i);
      data = json_array_get(vessels, i);
      if(!json_is_object(data)) {printf("error: not an object\n");}
      data = json_object_get(data, "localBoat");
      if(!json_is_object(data)) {printf("error: local boat not an object\n");}
      navigation = json_object_get(data, "navigation");
      if(!json_is_object(navigation)) {printf("error: navigation not an object\n");}
      data = json_object_get(navigation, "headingTrue");
      data = json_object_get(navigation, "rateOfTurn");
      if(!json_is_object(data)) {printf("error: heading true not an object\n");}
      data = json_object_get(data, "source");

      printf("%s\n",
           json_string_value(data));

    }

    return 0;
}

